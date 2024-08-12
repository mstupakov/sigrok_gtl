##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2024 Maksym Stupakov <maksym.stupakov@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from .gtl_common import *

RX = 0
TX = 1
rxtx_channels = ('RX', 'TX')

GTL_MARKER = 0x05

GTL_MSG_MARKER = (0, 1)
GTL_MSG_ID     = (0, 2)
GTL_MSG_DST    = (0, 2)
GTL_MSG_SRC    = (0, 2)
GTL_MSG_LEN    = (0, 2)
GTL_MSG_DATA   = (0,  )

FSM_INIT, FSM_MARKER, FSM_MSG_ID, FSM_MSG_DST, FSM_MSG_SRC, FSM_MSG_LEN, FSM_MSG_DATA = range(7)

class Decoder(srd.Decoder):
    api_version = 3
    id          = 'gtl'
    name        = 'GTL'
    longname    = 'Decoder for Dialog / Renesas GTL protocol'
    desc        = 'Dialog GTL decoder.'
    license     = 'mit'
    inputs      = ['uart', 'uart']
    outputs     = ['gtl']
    tags        = ['Embedded/industrial']

    annotations = (
        ('cs-msg-id', 'CS Message ID'), # 0
        ('cs-msg-dst', 'CS dst'),       # 1
        ('cs-msg-src', 'CS src'),       # 2
        ('cs-msg-len', 'CS len'),       # 3
        ('cs-msg-data', 'CS data'),     # 4
    )

    annotation_rows = (
        ('fields' , 'Fields', (0, 1, 2, 3, 4)),
        ('msgs', 'Messages', (0,)),
    )

    options = (
        {'id': 'gtl_max_data_len', 'desc': 'GTL Max Data Len', 'default': 1024,
         'values': (256, 1024, 2048, 4096, 65536)},
    )

    def __init__(self):
        self.reset()

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        pass

    def reset(self):
        self.msg_bytes = []
        self.ss_gtl_msg = 0
        self.es_gtl_msg = 0
        self.fsm = FSM_INIT
        self.prev_fsm = FSM_INIT

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        if ptype != 'DATA':
            return

        self.byte_len = es - ss

        # Returns data byte only
        pdata = pdata[0]

        if self.fsm == FSM_INIT:
            if pdata != GTL_MARKER:
                return
            else:
                self.ss_block, self.es_block = ss, es
                self.ss_gtl_msg = ss
                self.fsm = FSM_MARKER
                self.prev_fsm = FSM_INIT

        self.msg_bytes.append(pdata)

        def msg_data_to_int(value: tuple):
            str_value = '{1:02x}{0:02x}'.format(*value)
            return int(str_value, 16)

        if self.fsm != self.prev_fsm:
            self.ss_block = ss

        self.prev_fsm = self.fsm
        self.es_gtl_msg = es

        if self.fsm == FSM_MARKER:
            if (len(self.msg_bytes) == 1):
                self.msg_bytes = []
                self.fsm = FSM_MSG_ID

                self.put(self.ss_block, es, self.out_ann, [0, [f'GTL_MARKER']])

        elif self.fsm == FSM_MSG_ID:
            if (len(self.msg_bytes) == 2):
                msg_id = self.msg_bytes[slice(*(GTL_MSG_ID))]
                self.int_msg_id = msg_data_to_int(msg_id)

                self.msg_bytes = []
                self.fsm = FSM_MSG_DST

                self.str_msg_id = msg_id_db.get(self.int_msg_id, None)
                if self.str_msg_id == None:
                    self.fsm = FSM_INIT
                    self.str_msg_id = 'UNKNOWN MSG_ID'

                self.put(self.ss_block, es, self.out_ann,
                         [0, [f'MSG: {self.str_msg_id} ({hex(self.int_msg_id)})']])

        elif self.fsm == FSM_MSG_DST:
            if (len(self.msg_bytes) == 2):
                msg_dst = self.msg_bytes[slice(*(GTL_MSG_DST))]
                self.int_msg_dst = msg_data_to_int(msg_dst)

                self.msg_bytes = []
                self.fsm = FSM_MSG_SRC

                self.put(self.ss_block, es, self.out_ann, [0, [f'DST: {hex(self.int_msg_dst)}']])

        elif self.fsm == FSM_MSG_SRC:
            if (len(self.msg_bytes) == 2):
                msg_src = self.msg_bytes[slice(*(GTL_MSG_SRC))]
                self.int_msg_src = msg_data_to_int(msg_src)

                self.msg_bytes = []
                self.fsm = FSM_MSG_LEN

                self.put(self.ss_block, es, self.out_ann, [0, [f'SRC: {hex(self.int_msg_src)}']])

        elif self.fsm == FSM_MSG_LEN:
            if (len(self.msg_bytes) == 2):
                msg_len = self.msg_bytes[slice(*(GTL_MSG_LEN))]
                self.int_msg_len = msg_data_to_int(msg_len)

                self.msg_bytes = []

                if self.int_msg_len > self.options['gtl_max_data_len']: # Error condition
                    self.fsm = FSM_INIT

                else:
                    if self.int_msg_len != 0:
                        self.fsm = FSM_MSG_DATA
                    else:
                        self.fsm = FSM_INIT
                        self.put(self.ss_gtl_msg, self.es_gtl_msg, self.out_ann, [1, [f'{self.str_msg_id}']])

                self.put(self.ss_block, es, self.out_ann, [0, [f'LEN: {self.int_msg_len}']])

        elif self.fsm == FSM_MSG_DATA:
            if (len(self.msg_bytes) == self.int_msg_len):
                self.msg_data = self.msg_bytes[0:self.int_msg_len]

                self.msg_bytes = []
                self.fsm = FSM_INIT

                msg_data_hex = ' '.join(map('{:02x}'.format, self.msg_data))

                if hasattr(self, 'handle_' + self.str_msg_id):
                    func = getattr(self, 'handle_' + self.str_msg_id)
                    data = func(self.msg_data)
                    msg_data_hex = data

                self.put(self.ss_block, es, self.out_ann, [0, [f'DATA: {msg_data_hex}']])
                self.put(self.ss_gtl_msg, self.es_gtl_msg, self.out_ann, [1, [f'{self.str_msg_id}']])

                self.reset()


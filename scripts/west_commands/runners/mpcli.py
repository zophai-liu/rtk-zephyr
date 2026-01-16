# Copyright (c) 2017 Linaro Limited.
# Copyright (c) 2026, Realtek Semiconductor Corporation.
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing bee devices with mpcli.'''

import os
import json
from typing import Any, Dict, List
from pathlib import Path
from textwrap import dedent
from west import log

from runners.core import FileType, RunnerCaps, ZephyrBinaryRunner, BuildConfiguration

class MPCLIBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for espidf.'''

    def __init__(self, cfg, port,
                 build_dir, bin_address, chip_erase, mp_json, reset):
        super().__init__(cfg)
        self.port = port
        self.build_dir = build_dir
        self.bin_address = bin_address
        self.chip_erase = chip_erase
        self.mp_json = mp_json
        self.baud = "1000000"
        self.reset = reset
        self.elf = cfg.elf_file
        self.app_bin_file = cfg.bin_file
        self.ext_file = cfg.file
        self.ext_file_type = cfg.file_type
        self.files: List[Dict[str, Any]] = []

    @classmethod
    def name(cls):
        return 'mpcli'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, file=True, erase=True, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        mpcli_parser = parser
        mpcli_parser.add_argument('--port', required=True, type=str,
                                help='Serial communication port (e.g., COM3, /dev/ttyUSB0)')
        mpcli_parser.add_argument('--bin-address', type=str,
                        help='Download address(hex format, e.g., 0x8000000) for specified binary file ')
        mpcli_parser.add_argument('--mp-json', type=str,
                        help=dedent('''
                        Configuration json file containing binary path and download address.
                        Example format:
                        {
                            "mptoolconfig": {
                                "port": "",
                                "baud": "",
                                "appimage": {
                                    "relativepath": "",
                                    "file": [
                                        {
                                            "id": 0,
                                            "address": "0x00801000",
                                            "name": "fw1.bin",
                                            "enable": "1"
                                        }
                                    ]
                                }
                            }
                        }
                        '''))
        return parser

    @classmethod
    def do_create(cls, cfg, args):
        return MPCLIBinaryRunner(
            cfg, args.port, build_dir=cfg.build_dir, bin_address=args.bin_address, chip_erase=args.erase, mp_json=args.mp_json, reset=args.reset)

    def forceable_check(self, cond, msg=""):
        if not cond:
            log.die(msg)

    def export_to_file(self, filename: str) -> None:
        mptool_config = {
            "mptoolconfig": {
                "port": self.port,
                "baud": self.baud,
                "appimage": {
                    "relativepath": "",
                    "file": self.files
                }
            }
        }
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(mptool_config, f, indent=4, ensure_ascii=False)

    def add_file(self, address: str, name: str,id: int = 0, enable: str = "1") -> None:
        file_item = {
            "id": id,
            "address": address,
            "name": name,
            "enable": enable
        }
        self.files.append(file_item)

    def do_run(self, command, **kwargs):
        self.require('mpcli')
        bin_file_path = ""
        download_address = ""
        mptoolconfig_path = ""

        if self.mp_json:
            # use file provided by mp json
            mptoolconfig_path = self.mp_json
            if not os.path.isfile(mptoolconfig_path):
                log.err('no such json file {}'.format(mptoolconfig_path))
        else:
            if self.ext_file is not None:
                # use file provided by the user
                if self.ext_file_type == FileType.BIN:
                    bin_file_path = Path(self.ext_file)
                    if self.bin_address:
                        download_address = self.bin_address
                    else:
                        err = 'Cannot flash; --bin-address is required when file is specifiied'
                        raise ValueError(err)
                else:
                    err = 'Cannot flash; mpcli runner only supports bin file'
                    raise ValueError(err)
            else:
                # Use bin provided by the buildsystem.
                bin_file_path = Path(self.app_bin_file)
                if self.bin_address:
                    # Use address provided by the user.
                    download_address = self.bin_address
                else:
                    download_address = hex(self.flash_address_from_build_conf(self.build_conf))

            if not os.path.isfile(bin_file_path):
                log.err('Cannot flash; file ({}) not found'.format(bin_file_path))

            self.add_file(download_address, bin_file_path.name)

            mptoolconfig_path = str(bin_file_path.parent / "mptoolconfig.json")
            self.export_to_file(mptoolconfig_path)

        cmd_args = [
            'mpcli',
            '-c', self.port,
            '-f', mptoolconfig_path,
            '-a',
        ]

        if self.reset:
            cmd_args.extend(['-r'])

        if self.chip_erase:
            cmd_args.extend(["-E"])

        try:
            self.check_call(cmd_args)
        except Exception as e:
            self.logger.error(cmd_args)
            self.logger.error(e.args)

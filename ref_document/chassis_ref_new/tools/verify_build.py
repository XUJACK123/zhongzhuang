#!/usr/bin/env python3
"""Build the ARM target, retain log, and check weight placement in ELF."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import struct
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--fresh",action="store_true",help="Force compilation without deleting existing build artifacts")
    a=parser.parse_args()
    build=ROOT/"build/rl"; build.mkdir(parents=True,exist_ok=True)
    command=[shutil.which("make") or "make","-s","-j4"]
    if a.fresh: command.append("-B")
    result=subprocess.run(command,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (build/"build.log").write_text(result.stdout,encoding="utf-8")
    if result.returncode:
        print(result.stdout); raise SystemExit(result.returncode)
    elf=build/"wheel_leg_rl.elf"
    nm=subprocess.run(["arm-none-eabi-nm","-S",str(elf)],capture_output=True,text=True,check=True).stdout
    binary=elf.with_suffix('.bin').read_bytes()
    reset=[line.split() for line in nm.splitlines() if line.split()[-1:] == ['Reset_Handler']]
    if not reset or len(binary)<256:
        raise AssertionError('Missing Reset_Handler/vector table; startup assembly is invalid')
    stack,entry=struct.unpack_from('<II',binary)
    if not 0x20000000<stack<=0x20020000 or entry!=(int(reset[0][0],16)|1):
        raise AssertionError('Invalid stack/reset vector in binary')
    weights={}
    for line in nm.splitlines():
        parts=line.split()
        if len(parts)==4 and re.fullmatch(r"rl_[wb][0-3]",parts[3]):
            address,size=int(parts[0],16),int(parts[1],16)
            if not 0x08000000<=address<address+size<=0x08100000:
                raise AssertionError(f"Weight {parts[3]} is not in FLASH")
            weights[parts[3]]={"address":hex(address),"bytes":size,"symbol_type":parts[2]}
    if len(weights)!=8 or sum(v["bytes"] for v in weights.values())!=14630*4:
        raise AssertionError("Missing or wrong-sized actor weights in ELF")
    size=subprocess.run(["arm-none-eabi-size",str(elf)],capture_output=True,text=True,check=True).stdout.strip()
    usage=[]
    for p in build.rglob("*.su"):
        if "/user/" in p.as_posix():
            for line in p.read_text().splitlines():
                fields=line.split("\t")
                if len(fields)>=2: usage.append((int(fields[1]),fields[0]))
    warnings=[line for line in result.stdout.splitlines() if "warning:" in line]
    flags={name:int(value) for name,value in re.findall(r"#define ROBOT_(ENABLE_OUTPUT|MOTOR_CALIBRATED|IMU_FRAME_VERIFIED|POLICY_CONTRACT_VERIFIED|ODOMETRY_VERIFIED) ([01])",(ROOT/"user/robot_config.h").read_text())}
    report={"status":"pass","forced_rebuild":a.fresh,"production_commissioning_flags":flags,
            "arm_toolchain":subprocess.run(["arm-none-eabi-gcc","--version"],capture_output=True,text=True,check=True).stdout.splitlines()[0],
            "size":size,"weights_in_flash":weights,"max_single_user_function_stack_bytes":max(n for n,_ in usage),
            "warnings":warnings,"hardware_accessed":False,"board_timing_measured":False}
    (build/"build_report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    print(size)
    print(f"PASS: 14630 float32 weights in FLASH; largest single user-function stack {report['max_single_user_function_stack_bytes']} bytes")
    print(f"Warnings: {len(warnings)}; full build log: {build/'build.log'}")
if __name__=="__main__": main()

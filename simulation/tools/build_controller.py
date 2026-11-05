#!/usr/bin/env python3
"""Build the real Webots C++ motor controller, without a global build tool."""
import os
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HOME = Path(os.environ.get('WEBOTS_HOME', '/Applications/Webots.app/Contents'))
DEST = ROOT / 'simulation/webots/controllers/robot_driver'

def main():
    suffix = '.dylib' if platform.system() == 'Darwin' else '.so'
    command = ['clang++' if platform.system() == 'Darwin' else 'g++', '-std=c++17', '-O2',
               '-I'+str(HOME/'include/controller/cpp'), '-I'+str(HOME/'include/controller/c'),
               str(DEST/'robot_driver.cpp'), '-L'+str(HOME/'lib/controller'),
               '-lCppController', '-lController', '-Wl,-rpath,'+str(HOME/'lib/controller'),
               '-o', str(DEST/'robot_driver')]
    subprocess.run(command, check=True)
    print('Controlador Webots compilado:', DEST/'robot_driver')

if __name__ == '__main__': main()

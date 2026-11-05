"""Compile and execute the portable wheel controller, including braking signs."""
from pathlib import Path
import subprocess

def test_cpp_motor_limits_braking_and_encoder_signs(tmp_path):
    source=Path(__file__).with_name('test_control.cpp')
    binary=tmp_path/'control-test'
    subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)

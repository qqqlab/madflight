"""
documentationi generator script for platformio.
(c) 2026 madflight
"""

from os.path import join
import glob
import sys
import datetime
Import("env")

# Install yaml if missing
try:
    import yaml
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pyyaml")
    import yaml

platform = env.PioPlatform()
PROJ_SRC = env["PROJECT_SRC_DIR"]
PROJECT_DIR = env["PROJECT_DIR"]
CONFIG_FILES = glob.glob(join(PROJ_SRC, '*/param.yaml'), recursive=True)
outfilename =  PROJECT_DIR + "/Parameters.md"

print("==============================================")
print("madflight documentation generator")
print(f"Generating {outfilename}")
param = {}
for filename in CONFIG_FILES:
  print(f"  from {filename.removeprefix(PROJ_SRC)}")
  with open(filename) as stream:
    y = yaml.safe_load(stream)
    for k in y['param']:
      param[k] = y['param'][k]

  param = dict(sorted(param.items()))
  with open(outfilename, "w") as f:
    f.write(f"# Config Parameters\n")
    f.write(f"|Parameter|Default|Options|Comment|\n")
    f.write(f"|-|-|-|-|\n")
    for k in param:
       p = param[k]
       if 'options' in p:
         options = ", ".join(p['options'])
       else:
         options = ""
       f.write(f"|{k}|{p['default']}|{options}|{p.get('comment','')}|\n")


print("==============================================")

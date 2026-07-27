"""
parameter generator script for platformio.
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

def generate_config(outfilename, y):
  with open(outfilename, "w") as f:
    param = y['param']
    mf_options = {}
    for k in param:
      p = param[k]
      if 'options' in p:
        mf_options[k] = "mf_" + ",mf_".join(p['options'])
      else:
        mf_options[k] = ""

    f.write(f"// DO NOT EDIT - GENERATED ON {datetime.datetime.now()} BY extras/scripts/param_generator.py\n")
    f.write("\n")
    f.write("#pragma once\n")
    f.write("\n")
    f.write("#include <stdint.h>\n")
    f.write("\n")

    f.write("namespace Cfg {\n")

    f.write(f"  const uint16_t param_cnt = {len(param)}; //number of parameters\n")
    f.write("\n")

    f.write("  //enums for madflight library parameters (prefixed with 'mf_' to prevent macro expansion of global #defines like ADC or ICM45686)\n")
    for k in param:
      p = param[k]
      if p['type'] == 'e':
        f.write(f"  enum class {k}_enum : uint32_t {{ {mf_options[k]} }};\n")
        param[k]['datatype'] = f"Cfg::{k}_enum" #change datatype to enum
        param[k]['default'] =  f"Cfg::{k}_enum::mf_" + str(param[k]['default']) #expand default to enum key
    f.write("\n")

    f.write("  //list of parameters\n")
    f.write("  struct param_list_t {\n")
    f.write("    const char* name;\n")
    f.write("    const float defval;\n")
    f.write("    const char type;\n")
    f.write("    const char* options;\n")
    f.write("  };\n")
    f.write("  const param_list_t param_list[] = {\n")
    for k in param:
      p = param[k]
      f.write(f"    {{ \"{k}\", (float){p['default']}, '{p['type']}', \"{mf_options[k]}\" }},\n")
    f.write("  }; //const param_list_t param_list[]\n")

    f.write("}; //namespace Cfg\n")
    f.write("\n")

    f.write("//all parameters with defaults\n")
    f.write("struct CfgParam {\n")
    for k in param:
      p = param[k]
      if 'since' in p:
        f.write(f"  // {p['since']}\n")
      comment = ""
      if 'comment' in p: 
        comment = " // " + p['comment']
      f.write(f"  {p['datatype']} {k} = {p['default']};{comment}\n")
    f.write("}; //struct CfgParam\n")
    f.write("\n")



platform = env.PioPlatform()
PROJ_SRC = env["PROJECT_SRC_DIR"]
CONFIG_FILES = glob.glob(join(PROJ_SRC, '*/param.yaml'), recursive=True)

print("==============================================")
print("madflight parameter generator")
#print("PROJ_SRC:", PROJ_SRC)

for filename in CONFIG_FILES:
  print(f"Generating {filename.removeprefix(PROJ_SRC)}.h from {filename.removeprefix(PROJ_SRC)}")
  with open(filename) as stream:
    y = yaml.safe_load(stream)
    #print(y)
    generate_config(filename + ".h", y)

print("==============================================")

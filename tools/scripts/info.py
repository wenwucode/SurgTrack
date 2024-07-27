import yaml
import os

count = len(os.listdir("./rgb"))
for i in range(2000,2751):
    list = [] #3*3
    d={
        i:{
            "cam_K": list,
            "depth_scale": 1.0,
        }
    }
    f=open("info.yml","a",encoding="utf-8")
    yaml.dump(d,f)
    f.close()



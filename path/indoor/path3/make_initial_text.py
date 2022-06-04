#!/usr/bin/env python3
print("<!-- initial pose-->\n<setup>")
for i in range(5):
    f_0 = open("uav{0}.txt".format(i), "r")
    d_0 = f_0.read()
    d_0 = d_0.split("\n")
    d_0 = d_0[3]
    d_0 = d_0.split("\t")
    print("\t<robot>\n\t\t<name>uav{6}</name>\n\t\t<pose>\n\t\t\t<x>{0}</x>\n\t\t\t<y>{1}</y>\n\t\t\t<z>{2}</z>\n\t\t\t<R>{3}</R>\n\t\t\t<P>{4}</P>\n\t\t\t<Y>{5}</Y>\n\t\t</pose>\n\t</robot>\n".format(d_0[0], d_0[1], d_0[2], 0, 0, d_0[3],i))

print("</setup>")
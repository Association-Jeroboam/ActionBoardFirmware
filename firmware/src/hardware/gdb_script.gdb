target extended-remote localhost:3333
mon speed 10000
mon reset 0
layout split
focus cmd

#break can_lld_transmit
break hal_can_lld.c:418

[schematic2]
uniq 25
[tools]
[detail]
w 592 3331 100 0 n#1 inhier.Mode.P 320 3328 864 3328 864 3216 922 3216 eecsmotor.eecsmotor#14.MODE
w 584 3283 100 0 n#2 inhier.Tolerance.P 320 3280 848 3280 848 3152 922 3152 eecsmotor.eecsmotor#14.MDBD
w 748 3187 100 0 n#3 elongins.elongins#5.VAL 680 3184 816 3184 816 3096 922 3096 eecsmotor.eecsmotor#14.HINP
w 748 3043 100 0 n#4 elongins.elongins#7.VAL 680 3040 816 3040 816 3064 922 3064 eecsmotor.eecsmotor#14.RRBV
w 1429 3155 100 0 n#5 eecsmotor.eecsmotor#14.HSTA 1242 3152 1616 3152 elongouts.elongouts#8.DOL
w 1373 3123 100 0 n#6 eecsmotor.eecsmotor#14.RPOS 1242 3120 1504 3120 1504 2992 1616 2992 elongouts.elongouts#10.DOL
w 488 2979 100 0 n#7 inhier.Fault.P 328 2976 648 2976 648 3024 922 3024 eecsmotor.eecsmotor#14.FLT
w 508 2915 100 0 n#8 inhier.Simulation.P 328 2912 688 2912 688 2992 922 2992 eecsmotor.eecsmotor#14.SIML
w 524 2851 100 0 n#9 inhier.Debug.P 328 2848 720 2848 720 2960 920 2960 eecsmotor.eecsmotor#14.DBGL
w 564 2787 100 0 n#10 inhier.Slink.P 328 2784 800 2784 800 2896 920 2896 eecsmotor.eecsmotor#14.SLNK
w 1272 3219 100 0 n#11 eecsmotor.eecsmotor#14.DSTL 1240 3216 1304 3216 1304 3360 2128 3360 outhier.Response.p
w 1340 3187 100 0 n#12 eecsmotor.eecsmotor#14.MSGL 1240 3184 1440 3184 1440 3296 2128 3296 outhier.Message.p
w 1356 3059 100 0 n#13 eecsmotor.eecsmotor#14.MPOS 1240 3056 1472 3056 1472 2672 1920 2672 1920 2704 2128 2704 outhier.DevPosn.p
w 1344 3027 100 0 n#14 eecsmotor.eecsmotor#14.DMOV 1240 3024 1448 3024 1448 2640 2128 2640 outhier.InPosn.p
w 1336 2995 100 0 n#15 eecsmotor.eecsmotor#14.MIP 1240 2992 1432 2992 1432 2560 2128 2560 outhier.Status.p
w 1324 2963 100 0 n#16 eecsmotor.eecsmotor#14.HLS 1240 2960 1408 2960 1408 2480 2128 2480 outhier.HLimit.p
w 1312 2931 100 0 n#17 eecsmotor.eecsmotor#14.LLS 1240 2928 1384 2928 1384 2384 2128 2384 outhier.LLimit.p
w 1300 2899 100 0 n#18 eecsmotor.eecsmotor#14.FLNK 1240 2896 1360 2896 1360 2304 2128 2304 outhier.Flink.p
w 1364 3091 100 0 n#19 eecsmotor.eecsmotor#14.RVEL 1240 3088 1488 3088 1488 2832 1616 2832 elongouts.elongouts#36.DOL
w 372 3235 100 0 n#20 inhier.Handshake.P 320 3232 424 3232 elongins.elongins#5.INP
w 372 3091 100 0 n#21 inhier.Encoder.P 320 3088 424 3088 elongins.elongins#7.INP
w 1996 3091 100 0 n#22 elongouts.elongouts#8.OUT 1872 3088 2120 3088 outhier.HandshakeStatus.p
w 2000 2771 100 0 n#23 elongouts.elongouts#36.OUT 1872 2768 2128 2768 outhier.VelocityDemand.p
w 2000 2931 100 0 n#24 elongouts.elongouts#10.OUT 1872 2928 2128 2928 outhier.PositionDemand.p
[cell use]
use inhier 166 3321 100 0 Mode
xform 0 320 3328
use inhier 164 3273 100 0 Tolerance
xform 0 320 3280
use elongins 424 3152 100 0 elongins#5
xform 0 552 3200
p 284 3448 100 0 0 DTYP:$(dtyp)
p 479 3136 100 1024 -1 name:$(top)handshake
use elongins 253 3068 100 0 elongins#7
xform 0 552 3056
p 482 2991 100 1024 -1 name:$(top)encoder
p 284 3304 100 0 0 DTYP:$(dtyp)
use elongouts 1616 3056 100 0 elongouts#8
xform 0 1744 3120
p 1678 3041 100 1024 -1 name:$(top)hskstat
p 1572 3512 100 0 0 DTYP:$(dtyp)
p 1796 3060 100 0 1 OMSL:closed_loop
use elongouts 1616 2896 100 0 elongouts#10
xform 0 1744 2960
p 1572 3352 100 0 0 DTYP:$(dtyp)
p 1679 2878 100 1024 -1 name:$(top)posdmd
p 1799 2901 100 0 1 OMSL:closed_loop
use inhier 162 2970 100 0 Fault
xform 0 328 2976
use inhier 164 2906 100 0 Simulation
xform 0 328 2912
use eecsmotor 768 1208 100 0 eecsmotor#14
xform 0 1544 2272
use inhier 162 2844 100 0 Debug
xform 0 328 2848
use inhier 165 2778 100 0 Slink
xform 0 328 2784
use outhier 2172 3352 100 0 Response
xform 0 2112 3360
use outhier 2174 3291 100 0 Message
xform 0 2112 3296
use outhier 2172 2696 100 0 DevPosn
xform 0 2112 2704
use outhier 2172 2632 100 0 InPosn
xform 0 2112 2640
use outhier 2172 2552 100 0 Status
xform 0 2112 2560
use outhier 2172 2472 100 0 HLimit
xform 0 2112 2480
use outhier 2172 2376 100 0 LLimit
xform 0 2112 2384
use outhier 2175 2296 100 0 Flink
xform 0 2112 2304
use elongouts 1616 2736 100 0 elongouts#36
xform 0 1744 2800
p 1679 2718 100 1024 -1 name:$(top)veldmd
p 1572 3192 100 0 0 DTYP:$(dtyp)
p 1801 2743 100 0 1 OMSL:closed_loop
use inhier 164 3225 100 0 Handshake
xform 0 320 3232
use inhier 164 3081 100 0 Encoder
xform 0 320 3088
use outhier 2166 3083 100 0 HandshakeStatus
xform 0 2104 3088
use outhier 2174 2923 100 0 PositionDemand
xform 0 2112 2928
use outhier 2174 2763 100 0 VelocityDemand
xform 0 2112 2768
[comments]

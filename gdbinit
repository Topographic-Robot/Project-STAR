target remote :3333
set remote hardware-watchpoint-limit 2
mon reset hal
maintenance flush register-cache
thb app_main
c

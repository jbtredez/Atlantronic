include src/disco/lib_os.mk
include src/disco/lib_usb.mk
include src/disco/lib_cpu_board.mk
include src/disco/lib_baz.mk
include src/disco/lib_robot.mk
include src/disco/star/star.mk
include src/disco/gate/gate.mk

obj-disco-test_os += ${lib-os}
bin-disco += test_os

obj-disco-test_cpu_board += ${lib-os}
obj-disco-test_cpu_board += ${lib-usb}
obj-disco-test_cpu_board += ${lib-cpu_board}
bin-disco += test_cpu_board

obj-disco-test_baz += ${lib-os}
obj-disco-test_baz += ${lib-usb}
obj-disco-test_baz += ${lib-cpu_board}
obj-disco-test_baz += ${lib-baz}
bin-disco += test_baz

#obj-disco-test_robot += ${lib-os}
#obj-disco-test_robot += ${lib-usb}
#obj-disco-test_robot += ${lib-cpu_board}
#obj-disco-test_robot += ${lib-baz}
#obj-disco-test_robot += $(core_robot)
#bin-disco += test_robot

obj-disco-homologation_star += ${lib-os}
obj-disco-homologation_star += ${lib-usb}
obj-disco-homologation_star += ${lib-cpu_board}
obj-disco-homologation_star += $(lib-baz)
obj-disco-homologation_star += $(core_robot)
obj-disco-homologation_star += $(star_robot)
obj-disco-homologation_star += disco/strat/strat_simple.o
obj-disco-homologation_star += disco/star/strat_homol.o
bin-disco += homologation_star

obj-disco-homologation_gate += ${lib-os}
obj-disco-homologation_gate += ${lib-usb}
obj-disco-homologation_gate += ${lib-cpu_board}
obj-disco-homologation_gate += ${lib-baz}
obj-disco-homologation_gate += $(core_robot)
obj-disco-homologation_gate += $(gate_robot)
obj-disco-homologation_gate += disco/strat/strat_simple.o
obj-disco-homologation_gate += disco/gate/strat_homol.o
bin-disco += homologation_gate
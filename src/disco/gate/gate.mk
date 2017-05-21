gate_robot += disco/gate/gate.o
gate_robot += disco/gate/power.o
gate_robot += disco/gate/servos.o
gate_robot += kernel/PwmMotor.o
gate_robot += kernel/driver/encoder/EncoderSimulFromKinematicsModel.o
gate_robot += kernel/driver/StepperDriver.o
gate_robot += disco/gate/recalage.o

gate_robot += disco/gate/action/escapeStart.o
gate_robot += disco/gate/action/rocket_dismantler.o
gate_robot += disco/gate/action/drop_module.o
gate_robot += disco/gate/action/module_harvest.o

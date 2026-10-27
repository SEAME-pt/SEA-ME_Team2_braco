# mk/net.mk - is ROS 2 working, here and across the network? net-test net-talk net-listen.
#
# docker/tests/talker-listener.sh, in the `dev` container: a C++ talker and a Python listener, PASS/FAIL.
# Options (TOPIC, COUNT, TIMEOUT) are honoured only when given on the command line, because names this generic
# may well exist as unrelated environment variables: make net-listen COUNT=20 TIMEOUT=60 TOPIC=/arm_test

TL_ENV := $(foreach v,TOPIC COUNT TIMEOUT,$(if $(filter command line,$(origin $(v))),-e $(v)=$($(v))))
# The two-machine test meets on the real arm's domain: the PC's dev is on PC_ROS_DOMAIN_ID otherwise
TL_DOMAIN := -e ROS_DOMAIN_ID=$(ARM_ROS_DOMAIN_ID)
TL      = $(call dev_cmd,$(TL_ENV)) bash $(CONTAINER_TESTS)/talker-listener.sh

.PHONY: net-test net-talk net-listen

##@ Network (ROS 2 between the PC and the Jetson)

net-test: _image _dev_up ## Self-test in this machine's dev container: C++ talker + Python listener, PASS/FAIL
	$(TL) selftest

net-talk: _image _dev_up ## Two-machine test, step 1: the C++ talker here (Ctrl-C stops it)
	$(call dev_cmd,$(TL_ENV) $(TL_DOMAIN)) bash $(CONTAINER_TESTS)/talker-listener.sh talker

net-listen: _image _dev_up ## Two-machine test, step 2: on the OTHER machine, the checked listener (COUNT=0: no check)
	$(call dev_cmd,$(TL_ENV) $(TL_DOMAIN)) bash $(CONTAINER_TESTS)/talker-listener.sh listener

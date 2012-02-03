obj-linux-test_segment_intersection += kernel/math/segment_intersection.o
obj-linux-test_segment_intersection += linux/test/math/test_segment_intersection.o
lib-linux-test_segment_intersection += -lm
bin-linux += test_segment_intersection

obj-linux-test_trigo += kernel/math/trigo.o
obj-linux-test_trigo += linux/test/math/test_trigo.o
lib-linux-test_trigo += -lm
bin-linux += test_trigo

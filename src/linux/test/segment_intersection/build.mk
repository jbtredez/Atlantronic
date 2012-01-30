obj-linux-test_segment_intersection += kernel/math/segment_intersection.o
obj-linux-test_segment_intersection += linux/test/segment_intersection/test_segment_intersection.o

lib-linux-test_segment_intersection += -lm

bin-linux += test_segment_intersection

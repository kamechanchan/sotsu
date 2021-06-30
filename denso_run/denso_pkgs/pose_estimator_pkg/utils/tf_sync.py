#/usr/bin/env python3

import rospy

import queue
import tf 
from message_filters import SimpleFilter



class TfMessageFilter(SimpleFilter):
    """Stores a message unless corresponding transforms is 
    available
    """
    def __init__(self, input_filter, base_frame, target_frame,
                 queue_size=500):
        SimpleFilter.__init__(self)
        self.connectInput(input_filter)
        
        self.base_frame = base_frame
        self.target_frame = target_frame
        self.message_queue = queue.Queue(maxsize=queue_size)
        self.listener = tf.TransformListener()
        self.max_queue_size = queue_size
        self._max_queue_size_so_far = 0

    def connectInput(self, input_filter):
        self.incoming_connection = \
                input_filter.registerCallback(self.input_callback)

    def poll_transforms(self, latest_msg_tstamp):
        """
        Poll transforms corresponding to all messages. If found throw older
        messages than the timestamp of transform just found
        and if not found keep all the messages.
        """
        # Check all the messages for transform availability
        
        tmp_queue = queue.Queue(self.max_queue_size)
        first_iter = True
        get_shori_2 = rospy.get_param("/shori_2")
        get_shori_2 = get_shori_2 + 1
        #print("poll_transform is " + str(get_shori_2))
        rospy.set_param("/shori_2", get_shori_2)
        # Loop from old to new
        while not self.message_queue.empty():
            msg = self.message_queue.get()
            tstamp = msg.header.stamp
            if (first_iter and 
                self.message_queue.qsize() > self._max_queue_size_so_far):
                first_iter = False
                self._max_queue_size_so_far = self.message_queue.qsize()
                rospy.logdebug("Queue(%d) time range: %f - %f" %
                              (self.message_queue.qsize(), 
                               tstamp.secs, latest_msg_tstamp.secs))
                rospy.loginfo("Maximum queue size used:%d" %
                              self._max_queue_size_so_far)
            if self.listener.canTransform(self.base_frame, self.target_frame,
                                          tstamp):
                (trans, quat) = self.listener.lookupTransform(self.base_frame,
                                              self.target_frame, tstamp)
                self.signalMessage(msg, (trans, quat))
                get_shori_4 = rospy.get_param("/shori_4")
                get_shori_4 = get_shori_4 + 1
                print("signal message is " + str(get_shori_4))
                rospy.set_param("/shori_4", get_shori_4)
                
                # Note that we are deliberately throwing away the messages
                # older than transform we just received
                return
            else:
                # if we don't find any transform we will have to recycle all
                # the messages
                tmp_queue.put(msg)
        self.message_queue = tmp_queue

    def input_callback(self, msg):
        """ Handles incoming message """
        if self.message_queue.full():
            # throw away the oldest message
            rospy.logwarn("Queue too small. If you this message too often"
                          + " consider increasing queue_size")
            self.message_queue.get()

        self.message_queue.put(msg)
        get_shori_1 = rospy.get_param("/shori_1")
        get_shori_1 = get_shori_1 + 1
        print("input_callback is " + str(get_shori_1))
        rospy.set_param("/shori_1", get_shori_1)
        # This can be part of another timer thread
        # TODO: call this only when a new/changed transform
        self.poll_transforms(msg.header.stamp)


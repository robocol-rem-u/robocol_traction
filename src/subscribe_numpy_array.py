#!/usr/bin/env python
import rospy
import roslib
import numpy as np
from std_msgs.msg import Float32MultiArray
roslib.load_manifest('rospy')
from std_msgs.msg import MultiArrayDimension


def _serialize_numpy(self, buff):
    """
    wrapper for factory-generated class that passes numpy module into serialize
    """
    # pass in numpy module reference to prevent import in auto-generated code
    if self.layout.dim == []:
        self.layout.dim = [ MultiArrayDimension('dim%d' %i, self.data.shape[i], self.data.shape[i]*self.data.dtype.itemsize) for i in range(len(self.data.shape))];
    self.data = self.data.reshape([1, -1])[0];
    return self.serialize_numpy(buff, np)

def _deserialize_numpy(self, str):
    """
    wrapper for factory-generated class that passes numpy module into deserialize    
    """
    # pass in numpy module reference to prevent import in auto-generated code
    self.deserialize_numpy(str, np)
    dims=map(lambda x:x.size, self.layout.dim)
    self.data = self.data.reshape(dims)
    return self

## Use this function to generate message instances using numpy array
## types for numerical arrays. 
## @msg_type Message class: call this functioning on the message type that you pass
## into a Publisher or Subscriber call. 
## @returns Message class
def numpy_nd_msg(msg_type):
    classdict = { '__slots__': msg_type.__slots__, '_slot_types': msg_type._slot_types,
                  '_md5sum': msg_type._md5sum, '_type': msg_type._type,
                  '_has_header': msg_type._has_header, '_full_text': msg_type._full_text,
                  'serialize': _serialize_numpy, 'deserialize': _deserialize_numpy,
                  'serialize_numpy': msg_type.serialize_numpy,
                  'deserialize_numpy': msg_type.deserialize_numpy
                  }

    # create the numpy message type
    msg_type_name = "Numpy_%s"%msg_type._type.replace('/', '__')
    return type(msg_type_name,(msg_type,),classdict)



if __name__ == '__main__':
  
  def cb(data):
     print "I heard\n",data.data

  rospy.init_node('Subscriber')
  rospy.Subscriber("robocol/ruta",numpy_nd_msg(Float32MultiArray), cb)
  rospy.spin()
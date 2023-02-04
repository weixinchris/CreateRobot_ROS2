import rclpy                  
from rclpy.node   import Node    
from sensor_msgs.msg import JointState

class JoinStateSub(Node):

    def __init__(self, name):
        super().__init__(name)                          
        self.sub = self.create_subscription(\
            JointState, 
            "Odrive_joint_topic",
            self.listener_callback, 10) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self,msg:JointState):                      # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info(f'Time Stamp: %s' %msg.header) 
        self.get_logger().info(f'Vel [0]: %f' %msg.velocity[0])
        self.get_logger().info(f'Vel [1]: %f' %msg.velocity[1])
        self.get_logger().info(f'Pos [0]: %f' %msg.position[0])
        self.get_logger().info(f'Pos [0]: %f' %msg.position[1])


def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = JoinStateSub("JoinState")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口
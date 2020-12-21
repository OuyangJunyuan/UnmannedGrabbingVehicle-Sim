#!/home/ou/software/anaconda3/envs dl
import rospy
import ros_numpy as rnp
import numpy as np
from sensor_msgs.msg import PointCloud2 as pc2
import time

import torch.utils.data
from torch.autograd import Variable
from pointnet.dataset import ShapeNetDataset
from pointnet.model import PointNetCls
import torch.nn.functional as F

num_points = 2500
num_classes = 16
path_model = '/home/ou/workspace/ros_ws/ugv_ws/src/classification/PointNet/utils/cls/cls_model_3.pth'


def ros2net_bridge(ros_msg):
    start_time = time.time()
    raw_data = rnp.numpify(ros_msg)
    #  按dtype数据的label进行行排序
    data = np.array(raw_data.tolist())  # 将dtype-element array 转换成float-ndarray，这部使用大约10ms时间。
    fused = data[:, :3]
    labels = data[:, 3]
    net_data, net_data_idx = [], []
    for i in range(int(max(labels) + 1)):
        temp = labels == int(i)
        net_data_idx.append(temp)

        temp = fused[temp]  # 抽取label，只要0.0006
        temp = temp[np.random.choice(len(temp), num_points, replace=True), :]  # 重采样至2500个
        temp = temp - np.expand_dims(np.mean(temp, axis=0), 0)  # 中心化
        dist = np.max(np.sqrt(np.sum(temp ** 2, axis=1)), 0)  # 单位化
        net_data.append(temp / dist)
    net_data = np.array(net_data)
    end_time = time.time()
    # 耗时约10ms
    print("cvt~ cost:{:.1f}ms ".format((end_time - start_time) * 1000),
          "lables:{}  shape:{}".format(int(max(labels)) + 1, net_data.shape))
    # 使用np.copy 否则raw_data和ros_msg是共享内存的，其.flags.writable无法被修改。
    return np.copy(raw_data), net_data, net_data_idx


def callback(data):
    start_time = time.time()

    np_pc, points, idx = ros2net_bridge(data)
    points = Variable(torch.from_numpy(points)).float()
    points = points.transpose(2, 1).cuda()  # 转换为(batch_size, channels, num)
    pred, _, _ = classifier(points)
    pred = torch.exp(pred.cpu()).topk(1, dim=1)
    np_pc.setflags(write=1)
    for i in range(len(pred[0])):
        np_pc['label'][idx[i]] = pred[1][i].data.numpy()[0]
    data = rnp.msgify(pc2, np_pc)
    data.header.frame_id = 'base_link'
    pub.publish(data)

    end_time = time.time()
    print("classification~ cost:{:.1f}ms ".format((end_time - start_time) * 1000))
    print('-------------------------------------')


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('obj_classify', anonymous=True)
    rospy.Subscriber("fused_lidar", pc2, callback)
    global pub
    pub = rospy.Publisher("classification", pc2, queue_size=20)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    # 网络加载
    classifier = PointNetCls(k=num_classes)  # 训练模型为16类
    classifier.cuda()
    classifier.load_state_dict(torch.load(path_model))
    classifier.eval()  # 测试模式

    pub = None
    listener()

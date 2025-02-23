##
# @Author: shanzr
# @Description: point info and move plan
# @Date: Created in 2022/7/28
##
import math
class PointData:
    #��һ�η�������,��ʼ����Ϊ����x��������
    veca=[1, 0]

    # ����Ϣ
    point_data = {
        "S": [22, 22],
        "S1":[80, 22],
        "A1":[80, 65],
        "A2": [258, 65],
        "O1":[190,65],
        "C1": [190, 230],
        "D": [50, 230],
        "D1":[50,145],
        "P": [80, 145],
        "G1":[80,210],
        "1": [132, 210],
        "G2":[80,47],
        "2": [167, 47],
        "G3":[80,223],
        "3": [239, 223],
    }

    # ��ȡ��λ��
    @classmethod
    def get_point_info(cls, name):
        return cls.point_data[name]

    # ��ȡ�н���ʽ make up plan
    @classmethod
    def mup(cls, pfrom, pto):
        vecb=[cls.point_data[pto][0] - cls.point_data[pfrom][0],cls.point_data[pto][1] - cls.point_data[pfrom][1]]

        mob = math.sqrt(vecb[0] * vecb[0] + vecb[1] * vecb[1])
        moa = math.sqrt(cls.veca[0] * cls.veca[0] + cls.veca[1] * cls.veca[1])

        dot=(vecb[0] * cls.veca[0] + vecb[1] * cls.veca[1])
        times=vecb[0]*cls.veca[1]-vecb[1]*cls.veca[0]

        if times==0:
            if dot<0:
                theta=180
            else:
                theta=0
        else:
            theta=math.acos(dot/moa/mob)/3.14*180
            if times>0:
                theta*=-1

        cls.veca=vecb
        my_plan = [theta, mob/100]
        return my_plan

def sim_move(list):
    print("ת��"+str(list[0])+"�ȣ�Ȼ��ֱ��"+str(list[1])+"��")

if __name__ == '__main__':
    #����Ϊ����ģ��
    sim_move(PointData.mup("S", "A"))
    sim_move(PointData.mup("A", "B"))
    sim_move(PointData.mup("B", "O"))
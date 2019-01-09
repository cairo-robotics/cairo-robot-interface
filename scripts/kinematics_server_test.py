from kinematics_interface.kinematics_servers import MoveitForwardKinematicsServer, MoveitInverseKinematicsServer,\
                                         SawyerForwardKinematicsServer,SawyerInverseKinematicsServer


def test_fk():
    joints = [0.763331, 0.415979, -1.728629, 1.482985,
                       -1.135621, -1.674347, -0.496337]
    mi_fksvc = MoveitForwardKinematicsServer()
    int_fkscv = SawyerForwardKinematicsServer()
    print("HELLO")
    print(mi_fksvc.call(joints))
    print(int_fkscv.call(joints))


if __name__ == "__main__":
    test_fk()
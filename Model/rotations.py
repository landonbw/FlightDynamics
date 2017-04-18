import numpy as np

def vehicle_2_body(vector, phi, theta, psi):
    """
    rotate a vector in the vehicle frame to the same vector in the body frame
    :param vector: the vector that is to be rotated
    :param phi: roll of the aircraft
    :param theta: pitch of the aircraft
    :param psi: yaw of the aircraft
    :return: the rotated vector
    """
    sphi = np.sin(phi)
    cphi = np.cos(phi)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    spsi = np.sin(psi)
    cpsi = np.cos(psi)

    rot_mat = np.array([[ctheta * cpsi, ctheta * spsi, -stheta],
                        [sphi * stheta * cpsi - cphi * spsi, sphi * stheta * sphi + cphi * cpsi, sphi * ctheta],
                        [cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta]])

    new_vector = np.dot(rot_mat, vector)
    return new_vector

def body_2_vehicle(vector, phi, theta, psi):
    """
    rotate a vector from the body frame to the vehicle frame
    lazy mode is to copy the rotation matrix from the vehicle_2_body
    method and just use the transpose of it to get the roation
    :param vector: the vector that is to be rotated
    :param phi: roll of the aircraft
    :param theta: pitch of the aircraft
    :param psi: yaw of the aircraft
    :return: the rotated vector
    """
    sphi = np.sin(phi)
    cphi = np.cos(phi)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    spsi = np.sin(psi)
    cpsi = np.cos(psi)
    rot_mat = np.array([[ctheta * cpsi, ctheta * spsi, -stheta],
                        [sphi * stheta * cpsi - cphi * spsi, sphi * stheta * sphi + cphi * cpsi, sphi * ctheta],
                        [cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta]])
    rot_mat_transpose = np.transpose(rot_mat)
    new_vector = np.dot(rot_mat_transpose, vector)
    return new_vector

def body_2_inertial_quaternion(vector, quaternion):
    """
    rotate a vector from the body frame to the inertial/vehicle frame as
    specified by the quaternion
    :param vector: the vector that is to be rotated to the inertial frame
    :param quaternion: the quaternion that specifies the body grame
    :return: the rotated vector
    """
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)
    body2inertial = np.matrix([[e1 ** 2 + e0 ** 2 - e2 ** 2 - e3 ** 2, 2 * (e1 * e2 - e3 * e0), 2 * (e1 * e3 + e2 * e0)],
                              [2 * (e1 * e2 + e3 * e0), e2 ** 2 + e0 ** 2 - e1 ** 2 - e3 ** 2, 2 * (e2 * e3 - e1 * e0)],
                              [2 * (e1 * e3 - e2 * e0), 2 * (e2 * e3 + e1 * e0), e3 ** 2 + e0 ** 2 - e1 ** 2 - e2 ** 2]])
    vector_inertial = np.dot(body2inertial, vector)
    return vector_inertial





if __name__ == '__main__':
    vector = np.matrix([[20],[21],[22]])
    phi = np.radians(100)
    theta = np.radians(50)
    psi = np.radians(50)

    body = vehicle_2_body(vector, phi, theta, psi)
    vehicle = body_2_vehicle(vector, phi, theta, psi)
    check = body_2_vehicle(vehicle_2_body(vector, phi, theta, psi), phi, theta, psi) - vector
    print(check)

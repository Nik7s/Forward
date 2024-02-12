// simple geometric approach assuming zero twists and offsets, neglecting ee orientation

// function for calculating forward kinematics of a 3dof robot manipulator with all revolute joints
// first joint rotates are the z axis, changing x and y coordinates
// second two joints are perpendicular to the first, changing the height of the end effector (z value), and also x and y due to geometric constraints
EndEffectorPosition calculateForwardKinematics(const LinkLengths& linkLengths, const JointAngles& angles) {
    EndEffectorPosition position;

    double theta1 = angles.theta1;
    double theta2 = angles.theta2;
    double theta3 = angles.theta3;

    // forward kinematics calculation based on simple geometric and trigonometric relations

    // end effector position in the xy plane
    // x and y coordinates changes are related to both joint 1 rotation around z axis, and joint 2 and 3 rotation around their axes
    position.x = linkLengths.a1 * cos(theta1) + linkLengths.a2 * cos(theta1 + theta2) + linkLengths.a3 * cos(theta1 + theta2 + theta3);
    position.y = linkLengths.a1 * sin(theta1) + linkLengths.a2 * sin(theta1 + theta2) + linkLengths.a3 * sin(theta1 + theta2 + theta3);

    // end effector position in the z axis
    // z coordinate changes are related to joint 2 and 3 rotation around their axes
    position.z = linkLengths.a1 * sin(theta3) + linkLengths.a2 * cos(theta3);

    return position;
}
import cv2
from buildhat import Motor
from camera import Camera
from tracking.track import track

left_motor = Motor('C')
right_motor = Motor('B')
camera_motor = Motor('D')
BASE_SPEED = 0

def rotate(rotation):
    """Rotates the robot with a rotation speed

    Args:
        rotation (float): the speed with which to rotate
    """   
    left_speed = rotation
    right_speed = rotation
    left_motor.start(left_speed)
    right_motor.start(right_speed)

def adjust_camera(speed):
    """Adjusts the camera angle according to a given speed

    Args:
        speed (float): The speed with which to adjust the camera angle
    """  
    camera_motor.set_default_speed(speed)

def pd_controller(p_value, d_value, optimal_value, new_value, old_error):
    """The PD-Controller

    Args:
        p_value (float): The p-value for the controller
        d_value (float): The d-value for the controller
        optimal_value (float): The optimal value for the PD-Controller
        new_value (float): The value with which to calculate the error
        old_error (float): The previous error

    Returns:
        (float, float): The new error value and the pd value
    """  
    error = optimal_value - new_value
    error_change = error - old_error

    p = p_value * error
    d = d_value * error_change
    pd = p + d

    return error, pd

def horizontal_pd_to_rotation(pd):
    """Converts the horizontal pd value to a rotation

    Args:
        pd (float): The PD-value

    Returns:
        float: the rotation
    """ 
    pd = int(pd)
    if pd > 1:
        pd = pd + BASE_SPEED
    elif pd < -1:
        pd = pd - BASE_SPEED
    rotation = int(min(100,max(-100,-pd)))
    return rotation

def vertical_pd_to_speed(pd):
    """Converts the vertical pd value to a speed

    Args:
        pd (float): The vertical pd value

    Returns:
        float: the speed calculated from the PD
    """    
    pd = int(pd)
    speed = min(100,max(-100,-pd))
    return speed


def main():
    """The main function
    """  
    # Initiate the camera
    camera = Camera()

    # Set the PD controller values
    p_value_horizontal = 0.2
    d_value_horizontal = 0.2

    p_value_vertical = 0.2
    d_value_vertical = 1

    # Initialise values to 0
    old_horizontal_error = 0
    old_vertical_error = 0

    # In the optimal situation our error is 0
    optimal_value = 0

    # Initialise values to 0
    vertical_distance = 0
    horizontal_distance = 0

    # Set default motor speeds to 0
    left_motor.set_default_speed(0)
    right_motor.set_default_speed(0)
    camera_motor.set_default_speed(0)

    # The main loop
    while True:
        # make a picture
        ret, frame = camera.read()

        # collect the face vector by tracking this picture
        face_vector = track(frame)

        # If there is a face, gather the distance to the edge of the frame on
        # the horizontal axis and the distance to the center of the frame for
        # the vertical axis
        if(face_vector):
            horizontal_distance, vertical_distance = face_vector.to_distance()
            if horizontal_distance > 0:
                horizontal_distance = -320 + horizontal_distance
            else:
                horizontal_distance = 320 + horizontal_distance
        # Otherwise set the distance to 0       
        else:
            vertical_distance = 0
            horizontal_distance = 0

        # Gather values from the PD controller
        old_horizontal_error, pd_horizontal = pd_controller(p_value_horizontal, d_value_horizontal, optimal_value, horizontal_distance, old_horizontal_error)
        old_vertical_error, pd_vertical = pd_controller(p_value_vertical, d_value_vertical, optimal_value, vertical_distance, old_vertical_error)

        # Convert PD-values to speeds
        rotation_horizontal = horizontal_pd_to_rotation(pd_horizontal)
        speed_vertical = vertical_pd_to_speed(pd_vertical)
        
        # Update the motor with the new speeds
        rotate(rotation_horizontal)
        camera_motor.start(speed_vertical)

        # If q was pressed quit the program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Disable the motors
    camera_motor.stop()
    left_motor.stop()
    right_motor.stop()
    
    # Release handle to the webcam
    camera.release()
    cv2.destroyAllWindows()

# start the program
if __name__ == "__main__":
    main()
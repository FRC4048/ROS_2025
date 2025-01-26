class CamTable():
    
    # transformation of cam relative to the robot. x, y, z is translation, roll, pitch, yaw is rotation.
    #
    # NOTE: a rotation of -90, 0, 90 is required to change the camera to FLU (Front Left Up)
    #       this is done because of how apriltag_ros publishes transformation
    #
    #       the rotation of camera relative to the robot has to be applied on top of that
    #       https://www.andre-gaschler.com/rotationconverter/ is helpfull....
    #
    # All units are meters and degrees!
        
    cam_table = [
      {"camid": "cam1" , "x": 0, "y": 0, "z": 0, "roll": -90, "pitch": 0, "yaw": -90},
      {"camid": "cam2" , "x": 0, "y": 0, "z": 0, "roll": -90, "pitch": 0, "yaw": -90},
      {"camid": "cam3" , "x": 0, "y": 0, "z": 0, "roll": -90, "pitch": 0, "yaw": -90},
    ]
    

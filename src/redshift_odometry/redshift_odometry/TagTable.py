class TagTable():


    # tag positions units are meters and degrees!
    tag_table = [
      {"tagid": 1 , "x": 16.687292, "y": 0.628142, "z": 1.4859, "roll": 90, "pitch": 0, "yaw": -135, "desc": "coral"},
      {"tagid": 2 , "x": 16.687292, "y": 7.414259999999999, "z": 1.4859, "roll": 90, "pitch": 0, "yaw": -45, "desc": "coral"},
      {"tagid": 3, "x": 11.49096, "y": 8.031733999999998, "z": 1.30175, "roll": 90, "pitch": 0, "yaw": 0, "desc": "processor"},
      {"tagid": 4 , "x": 9.276079999999999, "y": 6.132575999999999, "z": 1.8679160000000001, "roll": 0, "pitch": -90, "yaw": 120, "desc": "barge"},
      {"tagid": 5 , "x": 9.276079999999999, "y": 1.9098259999999998, "z": 1.8679160000000001, "roll": 0, "pitch": -90, "yaw": 120, "desc": "barge"},
      {"tagid": 6 , "x": 13.474446, "y": 3.3012379999999997, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": 30, "desc": "reef"},
      {"tagid": 7 , "x": 13.890498, "y": 4.0208200000000005, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": 90, "desc": "reef"},
      {"tagid": 8 , "x": 13.474446, "y": 4.740402, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": 150, "desc": "reef"},
      {"tagid": 9 , "x": 12.643358, "y": 4.740402, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": -150, "desc": "reef"},
      {"tagid": 10, "x": 12.227305999999999, "y": 4.0208200000000005, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": -90, "desc": "reef"},
      {"tagid": 11, "x": 12.643358, "y": 3.3012379999999997, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": -30, "desc": "reef"},
      {"tagid": 12, "x": 0.8613139999999999, "y": 0.628142, "z": 1.4859, "roll": 90, "pitch": 0, "yaw": 135, "desc": "coral"},
      {"tagid": 13, "x": 0.8613139999999999, "y": 7.414259999999999, "z": 1.4859, "roll": 90, "pitch": 0, "yaw": 45, "desc": "coral"},
      {"tagid": 14, "x": 8.272272, "y": 6.132575999999999, "z": 1.8679160000000001, "roll": 120, "pitch": 0, "yaw": -90, "desc": "barge"},
      {"tagid": 15, "x": 8.272272, "y": 1.9098259999999998, "z": 1.8679160000000001, "roll": 120, "pitch": 0, "yaw": -90, "desc": "barge"},
      {"tagid": 16, "x": 6.057646, "y": 0.010667999999999999, "z": 1.30175, "roll": 90, "pitch": 0, "yaw": 180, "desc": "processor"},
      {"tagid": 17, "x": 4.073905999999999, "y": 3.3012379999999997, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": -30, "desc": "reef"},
      {"tagid": 18, "x": 3.6576, "y": 4.0208200000000005, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": -90, "desc": "reef"},
      {"tagid": 19, "x": 4.073905999999999, "y": 4.740402, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": -150, "desc": "reef"},
      {"tagid": 20, "x": 4.904739999999999, "y": 4.740402, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": 150, "desc": "reef"},
      {"tagid": 21, "x": 5.321046, "y": 4.0208200000000005, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 90, "desc": "reef"},
      {"tagid": 22, "x": 4.904739999999999, "y": 3.3012379999999997, "z": 0.308102, "roll": 90, "pitch": 0, "yaw": 30, "desc": "reef"},
    ]
    

    world_tag_table = [
      {"tagid": 1 , "x": 16.697, "y": 0.655, "z": 1.35, "roll": 90, "pitch": 0, "yaw": -135, "desc": "coral"},
      {"tagid": 2 , "x": 16.697, "y": 7.396, "z": 1.35, "roll": 90, "pitch": 0, "yaw": -45, "desc": "coral"},
      {"tagid": 3, "x": 11.561, "y": 8.056, "z": 1.17, "roll": 90, "pitch": 0, "yaw": 0, "desc": "processor"},
      {"tagid": 4 , "x": 9.276, "y": 6.138, "z": 1.78, "roll": 0, "pitch": -90, "yaw": 120, "desc": "barge"},
      {"tagid": 5 , "x": 9.276, "y": 1.915, "z": 1.78, "roll": 0, "pitch": -90, "yaw": 120, "desc": "barge"},
      {"tagid": 6 , "x": 13.474, "y": 3.306, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 30, "desc": "reef"},
      {"tagid": 7 , "x": 13.890, "y": 4.026, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 90, "desc": "reef"},
      {"tagid": 8 , "x": 13.474, "y": 4.745, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 150, "desc": "reef"},
      {"tagid": 9 , "x": 12.643, "y": 4.745, "z": 0.17, "roll": 90, "pitch": 0, "yaw": -150, "desc": "reef"},
      {"tagid": 10, "x": 12.227, "y": 4.026, "z": 0.17, "roll": 90, "pitch": 0, "yaw": -90, "desc": "reef"},
      {"tagid": 11, "x": 12.643, "y": 3.306, "z": 0.17, "roll": 90, "pitch": 0, "yaw": -30, "desc": "reef"},
      {"tagid": 12, "x": 0.851, "y": 0.655, "z": 1.35, "roll": 90, "pitch": 0, "yaw": 135, "desc": "coral"},
      {"tagid": 13, "x": 0.851, "y": 7.396, "z": 1.35, "roll": 90, "pitch": 0, "yaw": 45, "desc": "coral"},
      {"tagid": 14 , "x": 8.272, "y": 6.138, "z": 1.78, "roll": 120, "pitch": 0, "yaw": -90, "desc": "barge"},
      {"tagid": 15 , "x": 8.272, "y": 1.915, "z": 1.78, "roll": 120, "pitch": 0, "yaw": -90, "desc": "barge"},
      {"tagid": 16, "x": 6.445, "y": -0.004, "z": 1.17, "roll": 90, "pitch": 0, "yaw": 180, "desc": "processor"},
      {"tagid": 17, "x": 4.074, "y": 3.306, "z": 0.17, "roll": 90, "pitch": 0, "yaw": -30, "desc": "reef"},
      {"tagid": 18, "x": 3.658, "y": 4.026, "z": 0.17, "roll": 90, "pitch": 0, "yaw": -90, "desc": "reef"},
      {"tagid": 19, "x": 4.074, "y": 4.745, "z": 0.17, "roll": 90, "pitch": 0, "yaw": -150, "desc": "reef"},
      {"tagid": 20, "x": 4.905, "y": 4.745, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 150, "desc": "reef"},
      {"tagid": 21, "x": 5.321, "y": 4.026, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 90, "desc": "reef"},
      {"tagid": 22, "x": 4.905, "y": 3.306, "z": 0.17, "roll": 90, "pitch": 0, "yaw": 30, "desc": "reef"},
    ]




    # 1, 2, 12, 13 - coral station height is 135cm (MIDDLE of tag) and centered on the coral station
    # 3, 16        - processor height is 117cm and centered above the opening in the processor wall
    # 6-11, 17-22  - reef height is 17cm and centered horizontally on the reef face
    # 4, 5, 14, 15 - barge height is 178cm and approximately centered above the middle cage angled 30 degrees from vertical
    # low numbers are red, high are blue
    

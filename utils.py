def calculate_centers(img_width, x1, x2):
    screen_center_x = img_width // 2
    person_center_x = (x1 + x2) // 2

    if person_center_x - 100 > screen_center_x:
        return 1
    elif person_center_x + 100 < screen_center_x:
        return -1
    else:
        return 0

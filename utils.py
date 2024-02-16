#
def calculate_centers(img_width, x1, x2):
    # Calculate the center of the screen (X-coordinate)
    screen_center_x = img_width // 2
    print(f"screen center: {screen_center_x}")

    # Calculate the center of the detected person's bounding box
    person_center_x = (x1 + x2) // 2
    print(f"person_center_x: {person_center_x}")

    if person_center_x - 100 > screen_center_x:
        print("Person moved to the left")
    elif person_center_x + 100 < screen_center_x:
        print("Person moved to the right")
    else:
        print("Person is at the center")

    return screen_center_x, person_center_x

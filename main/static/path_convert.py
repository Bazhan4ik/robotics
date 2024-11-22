with open("example.txt") as file:
  lines = [line.rstrip("\n") for line in file]

  new_file = open("converted_to_cm.txt", "x")

  for line in lines:
    if line == "endData":
      break

    x, y, theta = line.split(",")

    new_line = str(round(float(x) * 2.54, 3)) + ", " + str(round(float(y) * 2.54, 3)) + "," + theta + "\n"





    new_file.write(new_line)

  new_file.write("endData")
  new_file.close()

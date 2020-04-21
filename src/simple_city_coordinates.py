
# These are dictionaries with the coordinates for the road intersections, Parking Structure, bridge,
# trees between lane 1 and lane 2 (top left), and trees in the playground (block 10)

RoadIntersections = {(1,1): (120, 100), (1,2): (120, 45), (1,3): (120, 0), (1,4): (120,-45), (1,5): (120,-100),
				 (2,1): (110, 100), (2,2): (110, 45), (2,3): (110, 0), (2,4): (110,-45), (2,5): (110,-100),
				 (3,1): (45, 100), (3,2): (45, 45), (3,3): (45, 0), (3,4): (45,-45), (3,5): (45,-100),
				 (4,1): (-15, 100), (4,2): (-15, 45), (4,3): (-15, 0), (4,4): (-15,-45), (4,5): (-15,-100),
				 (5,1): (-45, 100), (5,2): (-45, 45), (5,3): (-45, 0), (5,4): (-45,-45), (5,5): (-41,-98)}

# Starts from the top left

TreesBetweenLanes = {1:(115, 86.00), 2:(115, 77.70), 3:(115, 69.50), 4:(115, 61.25), 5:(115, 53.00),
			   		 6:(115, 35.00), 7:(115, 28.25), 8:(115, 21.50), 9:(115, 14.75), 10:(115, 8.00),
			   		 11:(115, -10.00), 12:(115, -16.75), 13:(115, -23.50), 14:(115, -30.25), 15:(115, -37.00),
			   		 16:(115, -59.00), 17:(115, -67.25), 18:(115, -75.50), 19:(115, -83.75), 20:(115, -92.00)}

ParkingStructureEntrance = {1:(125,70), 2:(125,45)}

# Matrix starts at top left with first row being entrance coordinates near road and second row is the end of the bridge coordinates near parking
Bridge = {(1,1):(-49,0), (1,2):(-49,-45),
		  (2,1):(-64,0), (2,2):(-64,-45)}

# This is the center of bus parking structure (near bottom right) as well as center of the playground park (block 10)
BusParkingCenter = {-74, -23}
PlaygroundCenter = {15.0, 72.5}
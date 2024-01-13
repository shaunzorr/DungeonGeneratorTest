@tool
extends Node3D

@onready var grid_map : GridMap = $GridMap

@export var start : bool = false : set = set_start

var predefined_rooms: Array = [
	{"size": Vector3(3, 0, 2), "doors": [Vector3(1, 0, -1), Vector3(1, 0, 2)]}
]

func set_start(val:bool)->void:
	if Engine.is_editor_hint():
		generate()

@export_range(0,1) var survival_chance : float = 0.25
@export var border_size : int = 20 : set = set_border_size
func set_border_size(val : int)->void:
	border_size = val
	if Engine.is_editor_hint():
		visualize_border()

@export var room_number : int = 4
@export var room_margin : int = 1
@export var room_recursion : int = 15
@export var min_room_size : int = 2 
@export var max_room_size : int = 4

@export var gate_room_size_width : int = 12
@export var gate_room_size_height : int = 12
@export var gate_room_loc: Vector3 = Vector3.ZERO

@export_multiline var custom_seed : String = "" : set = set_seed 
func set_seed(val:String)->void:
	custom_seed = val
	seed(val.hash())

var room_tiles: Array[PackedVector3Array] # room_tiles[room_id] -> list of covered tiles
var room_doors: Array[PackedInt32Array] # room_doors[room_id] -> door indices
var door_positions: PackedVector3Array # door_positions[door_id] -> door's position
var door_rooms: PackedInt32Array # door_rooms[door_id] -> door's room_id

func add_room(tiles: PackedVector3Array) -> int:
	var room_id := room_tiles.size()
	room_tiles.append(tiles)
	room_doors.append(PackedInt32Array())
	return room_id

func add_door(room_id: int, door_position: Vector3) -> int:
	assert(room_id >= 0 and room_id < room_tiles.size()) # validate room_id
	var door_id := door_positions.size()
	door_positions.append(door_position)
	door_rooms.append(room_id)
	room_doors[room_id].append(door_id)
	return door_id

func visualize_border():
	print("visualize_border()")
	grid_map.clear()
	for i in range(-1,border_size+1):
		grid_map.set_cell_item( Vector3i(i,0,-1),3)
		grid_map.set_cell_item( Vector3i(i,0,border_size),3)
		grid_map.set_cell_item( Vector3i(border_size,0,i),3)
		grid_map.set_cell_item( Vector3i(-1,0,i),3)

func generate():
	print("generate()")
	room_tiles.clear()
	room_doors.clear()
	door_positions.clear()
	door_rooms.clear()
	
	var t : int = 0
	if custom_seed : 
		set_seed(custom_seed)
	
	visualize_border()
	
	# generate rooms
	for i in range(room_number):
		var index = 0
		var room = predefined_rooms[index]
		var width = int(room["size"].x)
		var height = int(room["size"].z)
		room['position'] = Vector3i(width, 0, height)

		# Calculate a random position for the room
		var start_pos : Vector3i 
		start_pos.x = randi() % (border_size - width)
		start_pos.z = randi() % (border_size - height)

		# Place the room at the calculated position
		place_predefined_room(room_recursion, room)
	print("room_doors = ", room_doors)
	connection_calculations()
	
func connection_calculations():
	print("connection_calculations()")
	var door_positions_2d : PackedVector2Array = []
	var del_graph : AStar2D = AStar2D.new()
	var mst_graph : AStar2D = AStar2D.new()
	
	for door_id: int in door_positions.size():
		var p := door_positions[door_id]
		door_positions_2d.append(Vector2(p.x,p.z))
		del_graph.add_point(door_id, Vector2(p.x,p.z))
		mst_graph.add_point(door_id, Vector2(p.x,p.z))
	
	var delaunay : Array = Array(Geometry2D.triangulate_delaunay(door_positions_2d))
	
	print("door_positions_2d = ", door_positions_2d)
	print("delaunay = ", delaunay)
	
	# convert delaunay to astar
	for i in delaunay.size()/3:
		var p1 : int = delaunay.pop_front()
		var p2 : int = delaunay.pop_front()
		var p3 : int = delaunay.pop_front()
		del_graph.connect_points(p1,p2)
		del_graph.connect_points(p2,p3)
		del_graph.connect_points(p1,p3)
	
	var visited_points : PackedInt32Array = []
	visited_points.append(randi() % door_positions.size())
	while visited_points.size() != mst_graph.get_point_count():
		# build a list of yet unconnected doorways to this one
		var possible_connections : Array[PackedInt32Array] = []
		for vp in visited_points:
			for door_id_to in del_graph.get_point_connections(vp):
				if !visited_points.has(door_id_to):
					var con : PackedInt32Array = [vp,door_id_to]
					possible_connections.append(con)
		
		assert(not possible_connections.is_empty())
		
		# find the shortest possible connection
		var connection : PackedInt32Array = possible_connections.pick_random()
		for pc in possible_connections:
			var pc_len = door_positions_2d[pc[0]].distance_squared_to(door_positions_2d[pc[1]])
			var connection_len = door_positions_2d[connection[0]].distance_squared_to(door_positions_2d[connection[1]])
			if pc_len < connection_len:
				connection = pc
		
		# mark next point as visited and add it to the mst
		visited_points.append(connection[1])
		mst_graph.connect_points(connection[0],connection[1])
		print("mst_graph.connect_points(%s, %s)" % [connection[0],connection[1]])
		del_graph.disconnect_points(connection[0],connection[1])
	
	# let random unconnected lines survive
	for p in del_graph.get_point_ids():
		for door_id_to in del_graph.get_point_connections(p):
			if door_id_to>p:
				var kill : float = randf()
				if survival_chance > kill :
					mst_graph.connect_points(p,door_id_to)
					
	create_hallways(mst_graph)
	
func create_hallways(hallway_graph):
	print("create_hallways(%s)" % [hallway_graph])
	
	var hallways : Array[PackedVector3Array] = []
	for door_id_from in hallway_graph.get_point_ids():
		for door_id_to in hallway_graph.get_point_connections(door_id_from):
			if door_id_to <= door_id_from:
				continue
			
			if door_rooms[door_id_from] == door_rooms[door_id_to]:
				continue
			
			var tile_from : Vector3 = door_positions[door_id_from]
			var tile_to : Vector3 = door_positions[door_id_to]
			var hallway : PackedVector3Array = [tile_from,tile_to]
			
			hallways.append(hallway)
	print("hallways = ", hallways)
	
	var astar : AStarGrid2D = AStarGrid2D.new()
	astar.size = Vector2i.ONE * border_size
	astar.update()
	astar.diagonal_mode = AStarGrid2D.DIAGONAL_MODE_NEVER
	astar.default_estimate_heuristic = AStarGrid2D.HEURISTIC_MANHATTAN
	
	for t in grid_map.get_used_cells_by_item(0):
		astar.set_point_solid(Vector2i(t.x,t.z))
	var _t : int = 0
	for h in hallways:
		_t +=1
		var pos_from : Vector2i = Vector2i(h[0].x,h[0].z)
		var pos_to : Vector2i = Vector2i(h[1].x,h[1].z)
		var hall : PackedVector2Array = astar.get_point_path(pos_from,pos_to)
		
		for t in hall:
			var pos : Vector3i = Vector3i(t.x,0,t.y)
			if grid_map.get_cell_item(pos) <0 and grid_map.get_cell_item(pos) != 2:
				grid_map.set_cell_item(pos,1)
				
func place_predefined_room(rec, new_room):
	print("place_predefined_room(%s, %s)" % [rec, new_room])
	if !rec > 0:
		return
		
	var width : int = int(new_room["size"].x)
	var height : int = int(new_room["size"].z)
	
	var start_pos : Vector3i 
	start_pos = Vector3i(randi() % (border_size - width + 2), 0, randi() % (border_size - height + 2))
	
	for r in range(-room_margin,height+room_margin):
		for door_id_to in range(-room_margin,width+room_margin):
			var pos : Vector3i = start_pos + Vector3i(door_id_to,0,r)
			if grid_map.get_cell_item(pos) == 0 or grid_map.get_cell_item(pos) == 2:
				place_predefined_room(rec-1, new_room)
				return

	var room : PackedVector3Array = []
	var dt: PackedVector3Array = []
	
	for r in height:
		for door_id_to in width:
			var pos : Vector3i = start_pos + Vector3i(door_id_to,0,r)
			grid_map.set_cell_item(pos, 0)
			room.append(pos)
	
	var room_id := add_room(room)
	
	for door_position in new_room["doors"]:
		var door_pos : Vector3i = start_pos + Vector3i(door_position.x, 0, door_position.z)
		grid_map.set_cell_item(door_pos, 2)
		add_door(room_id, door_pos)

	#var avg_x : float = start_pos.x + (float(width)/2)
	#var avg_z : float = start_pos.z + (float(height)/2)
	#var room_pos : Vector3 = Vector3(avg_x,0,avg_z)
	
	#room_positions.append(room_pos)
	#room_tiles.append(room)
	#room_doors.append(dt)
	
	



func _unhandled_key_input(event: InputEvent) -> void:
	if Engine.is_editor_hint():
		return
	if event.pressed:
		match event.physical_keycode:
			KEY_SPACE:
				generate()

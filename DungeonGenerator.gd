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

var room_tiles : Array[PackedVector3Array] = []
var room_positions : PackedVector3Array = []
var door_positions: PackedVector3Array = []
var door_tiles: Array[PackedVector3Array] = []

func visualize_border():
	grid_map.clear()
	for i in range(-1,border_size+1):
		grid_map.set_cell_item( Vector3i(i,0,-1),3)
		grid_map.set_cell_item( Vector3i(i,0,border_size),3)
		grid_map.set_cell_item( Vector3i(border_size,0,i),3)
		grid_map.set_cell_item( Vector3i(-1,0,i),3)

func generate():
	room_tiles.clear()
	room_positions.clear()
	door_positions.clear()
	
	var t : int = 0
	if custom_seed : 
		set_seed(custom_seed)
	
	visualize_border()
			
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
		
	connection_calculations()
	
func connection_calculations():
	var rpv2 : PackedVector2Array = []
	var del_graph : AStar2D = AStar2D.new()
	var mst_graph : AStar2D = AStar2D.new()
	
	for p in door_positions:
		rpv2.append(Vector2(p.x,p.z))
		del_graph.add_point(del_graph.get_available_point_id(),Vector2(p.x,p.z))
		mst_graph.add_point(mst_graph.get_available_point_id(),Vector2(p.x,p.z))
	
	var delaunay : Array = Array(Geometry2D.triangulate_delaunay(rpv2))
	
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
		var possible_connections : Array[PackedInt32Array] = []
		for vp in visited_points:
			for c in del_graph.get_point_connections(vp):
				if !visited_points.has(c):
					var con : PackedInt32Array = [vp,c]
					possible_connections.append(con)
					
		var connection : PackedInt32Array = possible_connections.pick_random()
		for pc in possible_connections:
			if rpv2[pc[0]].distance_squared_to(rpv2[pc[1]]) <\
			rpv2[connection[0]].distance_squared_to(rpv2[connection[1]]):
				connection = pc
		
		visited_points.append(connection[1])
		mst_graph.connect_points(connection[0],connection[1])
		del_graph.disconnect_points(connection[0],connection[1])
	
	var hallway_graph : AStar2D = mst_graph
	
	for p in del_graph.get_point_ids():
		for c in del_graph.get_point_connections(p):
			if c>p:
				var kill : float = randf()
				if survival_chance > kill :
					hallway_graph.connect_points(p,c)
					
	create_hallways(hallway_graph)
	
func create_hallways(hallway_graph):
	var hallways : Array[PackedVector3Array] = []
	for p in hallway_graph.get_point_ids():
		for c in hallway_graph.get_point_connections(p):
			if c>p:
				var room_from : PackedVector3Array = door_tiles[p]
				var room_to : PackedVector3Array = door_tiles[c]
				var tile_from : Vector3 = room_from[0]
				var tile_to : Vector3 = room_to[0]
				for t in room_from:
					if t.distance_squared_to(door_positions[c])<\
					tile_from.distance_squared_to(door_positions[c]):
						tile_from = t
				for t in room_to:
					if t.distance_squared_to(door_positions[p])<\
					tile_to.distance_squared_to(door_positions[p]):
						tile_to = t
				var hallway : PackedVector3Array = [tile_from,tile_to]
				hallways.append(hallway)
				if grid_map.get_cell_item(tile_from) == 0 and grid_map.get_cell_item(tile_to) == 0:
					pass
	
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
	if !rec > 0:
		return
		
	var width : int = int(new_room["size"].x)
	var height : int = int(new_room["size"].z)
	
	var start_pos : Vector3i 
	start_pos = Vector3i(randi() % (border_size - width + 2), 0, randi() % (border_size - height + 2))
	
	for r in range(-room_margin,height+room_margin):
		for c in range(-room_margin,width+room_margin):
			var pos : Vector3i = start_pos + Vector3i(c,0,r)
			if grid_map.get_cell_item(pos) == 0 or grid_map.get_cell_item(pos) == 2:
				place_predefined_room(rec-1, new_room,)
				return

	var room : PackedVector3Array = []
	var dt: PackedVector3Array = []
	
	for r in height:
		for c in width:
			var pos : Vector3i = start_pos + Vector3i(c,0,r)
			grid_map.set_cell_item(pos, 0)
			room.append(pos)
			
	for door_position in new_room["doors"]:
		var door_pos : Vector3i = start_pos + Vector3i(door_position.x, 0, door_position.z)
		grid_map.set_cell_item(door_pos, 2)
		door_positions.append(door_pos)
		room.append(door_pos)
		dt.append(door_pos)

	var avg_x : float = start_pos.x + (float(width)/2)
	var avg_z : float = start_pos.z + (float(height)/2)
	var room_pos : Vector3 = Vector3(avg_x,0,avg_z)
	
	room_positions.append(room_pos)
	room_tiles.append(room)
	door_tiles.append(dt)
	
	




import single_vw
import setting

solution = [[1.5760100460146989, 0.5878097898265315, 0.25156223581354165, 0.2622385781393921], [0.6925393493350454, 0.7490456983233422, 1.7779569446779966, 0.04276797939893817], [0.3554035108688698, 0.816687093108839, 0.08908287146965432, 1.7138805056009039], [0.22821197671317273, 0.480183290398287, 1.0755115868538223, 0.32409872567530784]]
vwlist,vwlinelist = single_vw.VW.set_virtual_wall(solution)
print(vwlist)
vertexlist1 = single_vw.Environment.set_vertex_list(vwlist, single_vw.CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]))
vertexlist2 = single_vw.Environment.set_vertex_list(vwlist, single_vw.CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]))
vertexlist3 = single_vw.Environment.set_vertex_list(vwlist, single_vw.CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]))
vertexlist4 = single_vw.Environment.set_vertex_list(vwlist, single_vw.CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))

car1_visgraph = single_vw.Execution.visibility_graph(vertexlist1, vwlinelist)
car2_visgraph = single_vw.Execution.visibility_graph(vertexlist2, vwlinelist)
car3_visgraph = single_vw.Execution.visibility_graph(vertexlist3, vwlinelist)
car4_visgraph = single_vw.Execution.visibility_graph(vertexlist4, vwlinelist)

car1_shortest_path, car1_shortest_length = single_vw.Execution.dijkstra(car1_visgraph)
car2_shortest_path, car2_shortest_length = single_vw.Execution.dijkstra(car2_visgraph)
car3_shortest_path, car3_shortest_length = single_vw.Execution.dijkstra(car3_visgraph)
car4_shortest_path, car4_shortest_length = single_vw.Execution.dijkstra(car4_visgraph)

print(car1_shortest_path)
print(car2_shortest_path)
print(car3_shortest_path)
print(car4_shortest_path)


#include <Eigen/Dense>
#include <Eigen/Sparse>
////建图:代理面邻接关系
		Eigen::Matrix<bool, -1, -1> matA;
		matA.resize(cur_id, cur_id);
		matA.setZero();
		for (int i = 0; i < proxy_faces.size(); ++i)   //对于每个代理
		{
			for (int j = 0; j < proxy_faces[i].size(); ++j)//对于每个面
			{
				for (auto h : mesh->halfedges(proxy_faces[i][j])) {   //对于proxy_faces[i][j]面的半边找到其对边的邻域面cur：
					auto cur = mesh->face(mesh->opposite(h));         //邻域面cur：
					if (id[cur] != i)
					{
						matA(i, id[cur]) = 1;
						matA(id[cur], i) = 1;
						//triplets.emplace_back(i, id[cur], 1);//邻域面的代理不是当前的代理，加入
					}

				}
			}
		}
 
		//实验二：区域融合
		for (int i = 0; i < matA.rows(); ++i)
		{
			std::vector<SurfaceMesh::Vertex> proxy_vertices;
			float min_fitting_distance = 1000.0;
			int target_proxy = -1;

			if (proxy_areas[i] > area_threshold)
				continue;

			else
				proxy_vertices.clear();
			    for (int j = 0; j < proxy_faces[i].size(); ++j) //对于每个代理内的三角面删除重复的顶点
			    {
				    for (auto vv : mesh->vertices(proxy_faces[i][j])) //三角面的顶点存入数组
				    {
					    proxy_vertices.push_back(vv);
				    }
			    }
			    sort(proxy_vertices.begin(), proxy_vertices.end());
			    proxy_vertices.erase(unique(proxy_vertices.begin(), proxy_vertices.end()), proxy_vertices.end());//删除重复的顶点
			
			    for (int j = 0; j < matA.cols(); ++j)//迭代找非零值==相邻代理
			    {
				    //i为当前代理
				    //j为邻近代理
					if (matA(i,j) == 1 && i != j)
					{
						float dist = 0.0;
						vec3 nn = proxy_normals[j];                                         //j代理的法向量
						vec3 aa = vpoint_[mesh->target(mesh->halfedge(proxy_faces[j][0]))]; //j代理内面上的一点
						for (int p = 0; p < proxy_vertices.size(); p++)
						{
							vec3 ap = vpoint_[proxy_vertices[p]] - aa;               //i代理内一点减去邻域j代理内一点

							dist += (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));
						}

						if (dist < min_fitting_distance)
						{
							min_fitting_distance = dist;
							target_proxy = j;//此代理为目标融合的邻近代理,要把i融入target_proxy内！！！！！！
						}
					}
			    }

			    if (min_fitting_distance < fitting_threshold && target_proxy != -1)
			    {
				//更新代理法向量proxy_normals？更新proxy_nearproxies！更新proxy_faces！更新id！
					
					for (int j = 0; j < matA.cols(); ++j)
					{
						if (matA(i, j) == 1 && i != j)
						{
							matA(j, target_proxy) = 1;
							matA(target_proxy, j) = 1;
							matA(i, j) = 0;
							matA(j, i) = 0;
							matA(i, i) = 0;
							
						}
					}

					for (int m = 0; m < (int)proxy_faces[i].size(); m++)//更新proxy_faces！更新id！
					{
						proxy_faces[target_proxy].push_back(proxy_faces[i][m]);
						id[proxy_faces[i][m]] = target_proxy;
					}
					proxy_faces[i].clear();

					proxy_areas[target_proxy] += proxy_areas[i];
			    }
			    else
			    {
				    for (int j = 0; j < matA.cols(); ++j)
				    {
						if (matA(i, j) == 1 && i != j)
						{
							matA(i, j) = 0;
							matA(j, i) = 0;
							matA(i, i) = 0;
						}
					   
				    }
				    for (int m = 0; m < (int)proxy_faces[i].size(); m++)//更新proxy_faces！更新id！
				    {
					    id[proxy_faces[i][m]] = -1;
				    }
				    proxy_faces[i].clear();
			    }

				//cur_id--;
		}

//后处理退化的三角形


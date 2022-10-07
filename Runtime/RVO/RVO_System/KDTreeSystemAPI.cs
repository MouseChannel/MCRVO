using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using FixedMath;
using Unity.Collections;
using UnityEngine.Profiling;
using Unity.Jobs;

//  [UpdateInGroup(typeof(CommandGroup))]
//  [UpdateAfter(typeof(KeepWalkingSystem))]
[DisableAutoCreation]

public partial class KDTreeSystem : WorkSystem
{


 


    /// <summary>
    /// 每个逻辑帧运行， Agent的位置需要即时更新
    /// </summary>
    public override void Work()
    {




        agents_.Clear();
        agentTree_.Clear();





        Entities.ForEach((Entity entity, in Agent agent) =>
        {
            agents_.Add(agent);
            agentTree_.Add(new AgentTreeNode { });
            agentTree_.Add(new AgentTreeNode { });

        }).WithoutBurst().Run();






        new BuildAgentTreeJob
        {
            agents_ = agents_,
            agentTree_ = agentTree_
        }.Run();


      



        new UpdateAgentJobParallel
        {


            agents = agents_,
            agentTree = agentTree_,
            obstacleVertices_ = obstacleVertices_,
            obstacleVerticesTree_ = obstacleVerticesTree_,
            obstacleVerticesTreeRoot = obstacleVerticesTreeRoot,

            ecbPara = ecbPara

        }.Schedule(agents_.Length, 4).Complete();












    }
    /// <summary>
    /// 场景内增加障碍物是调用
    /// </summary>
    public void UpdateObstacle()
    {
        UpdateObstacleVerticeTree();
        UpdateObstacleTree();


        void UpdateObstacleTree()
        {

            obstacles_.Clear();
            obstacleTree_.Clear();



            Entities.ForEach((Entity entity, in Obstacle obstacle) =>
            {
                obstacles_.Add(obstacle);
                obstacleTree_.Add(new ObstacleTreeNode { });
                obstacleTree_.Add(new ObstacleTreeNode { });

            }).WithoutBurst().Run();


            new BuildObstacleTreeJob
            {
                obstacles_ = obstacles_,
                obstacleTree_ = obstacleTree_

            }.Run();


        }



        void UpdateObstacleVerticeTree()
        {
            obstacleVertices_.Clear();
            obstacleVerticesTree_.Clear();

            Entities.ForEach((Entity entity, DynamicBuffer<PreObstacleVertice> obstacleVertices) =>
            {
                ObstacleVerticeCollect(obstacleVertices_, obstacleVertices);
            }).WithoutBurst().Run();





            InitObstacleVerticeTree(obstacleVertices_.Length);



            var buildObstacleJob = new BuildObstacleVerticeTreeJob
            {
                obstacleVertices_ = obstacleVertices_,
                obstacleVerticesTree_ = obstacleVerticesTree_,
                // obstacleVerticesTreeRoot_ = obstacleVerticesTreeRoot
            };
            buildObstacleJob.Run();

            obstacleVerticesTreeRoot = buildObstacleJob.obstacleVerticesTreeRoot;



        }
    }

    protected override void OnDestroy()
    {
        agents_.Dispose();
        agentTree_.Dispose();
        obstacleVertices_.Dispose();
        obstacleVerticesTree_.Dispose();

        obstacles_.Dispose();
        obstacleTree_.Dispose();



    }





    public void GetClosestAgent(FixedVector2 position, ref FixedInt rangeSq, ref int agentNo, int node = 0)
    {


        if (agentTree_[node].end_ - agentTree_[node].begin_ <= MAX_LEAF_SIZE)
        {
            for (int i = agentTree_[node].begin_; i < agentTree_[node].end_; ++i)
            {

                FixedInt distSq = FixedCalculate.Square(position - agents_[i].position_);
                //Find EnemyUnit
                if (distSq < rangeSq)
                {
                    rangeSq = distSq;
                    agentNo = agents_[i].id_;
                }
            }
        }
        else
        {
            FixedInt distSqLeft = FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].left_].minX_ - position.X)) + FixedCalculate.Square(FixedCalculate.Max(0, position.X - agentTree_[agentTree_[node].left_].maxX_)) + FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].left_].minY_ - position.Y)) + FixedCalculate.Square(FixedCalculate.Max(0, position.Y - agentTree_[agentTree_[node].left_].maxY_));
            FixedInt distSqRight = FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].right_].minX_ - position.X)) + FixedCalculate.Square(FixedCalculate.Max(0, position.X - agentTree_[agentTree_[node].right_].maxX_)) + FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].right_].minY_ - position.Y)) + FixedCalculate.Square(FixedCalculate.Max(0, position.Y - agentTree_[agentTree_[node].right_].maxY_));

            if (distSqLeft < distSqRight)
            {
                if (distSqLeft < rangeSq)
                {
                    GetClosestAgent(position, ref rangeSq, ref agentNo, agentTree_[node].left_);

                    if (distSqRight < rangeSq)
                    {
                        GetClosestAgent(position, ref rangeSq, ref agentNo, agentTree_[node].right_);
                    }
                }
            }
            else
            {
                if (distSqRight < rangeSq)
                {
                    GetClosestAgent(position, ref rangeSq, ref agentNo, agentTree_[node].right_);

                    if (distSqLeft < rangeSq)
                    {
                        GetClosestAgent(position, ref rangeSq, ref agentNo, agentTree_[node].left_);
                    }
                }
            }

        }

    }


    public void GetAreaAgents(FixedVector2 position, Vector4 areaRect, FixedInt rangeSq, int node, List<int> areaAgents)
    {


        if (agentTree_[node].end_ - agentTree_[node].begin_ <= MAX_LEAF_SIZE)
        {
            for (int i = agentTree_[node].begin_; i < agentTree_[node].end_; ++i)
            {

                InsertAreaAgents(agents_[i], areaRect, areaAgents);

            }
        }
        else
        {
            FixedInt distSqLeft = FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].left_].minX_ - position.X)) + FixedCalculate.Square(FixedCalculate.Max(0, position.X - agentTree_[agentTree_[node].left_].maxX_)) + FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].left_].minY_ - position.Y)) + FixedCalculate.Square(FixedCalculate.Max(0, position.Y - agentTree_[agentTree_[node].left_].maxY_));
            FixedInt distSqRight = FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].right_].minX_ - position.X)) + FixedCalculate.Square(FixedCalculate.Max(0, position.X - agentTree_[agentTree_[node].right_].maxX_)) + FixedCalculate.Square(FixedCalculate.Max(0, agentTree_[agentTree_[node].right_].minY_ - position.Y)) + FixedCalculate.Square(FixedCalculate.Max(0, position.Y - agentTree_[agentTree_[node].right_].maxY_));

            if (distSqLeft < distSqRight)
            {
                if (distSqLeft < rangeSq)
                {
                    GetAreaAgents(position, areaRect, rangeSq, agentTree_[node].left_, areaAgents);

                    if (distSqRight < rangeSq)
                    {
                        GetAreaAgents(position, areaRect, rangeSq, agentTree_[node].right_, areaAgents);
                    }
                }
            }
            else
            {
                if (distSqRight < rangeSq)
                {
                    GetAreaAgents(position, areaRect, rangeSq, agentTree_[node].right_, areaAgents);

                    if (distSqLeft < rangeSq)
                    {
                        GetAreaAgents(position, areaRect, rangeSq, agentTree_[node].left_, areaAgents);
                    }
                }
            }

        }
        void InsertAreaAgents(Agent agent, Vector4 areaRect, List<int> areaAgents)
        {

            var pos = agent.position_;
            bool Vaild(float a, float min, float max) => a > min && a < max;


            if (Vaild(pos.X.RawFloat, areaRect[0], areaRect[1]) &&
                        Vaild(pos.Y.RawFloat, areaRect[2], areaRect[3]))
            {
                areaAgents.Add(agent.id_);
            }
        }



    }





    public void GetClosestObstacle(FixedVector2 position, ref FixedInt rangeSq, ref int obstacleNo, int node)
    {


        if (obstacleTree_[node].end_ - obstacleTree_[node].begin_ <= MAX_LEAF_SIZE)
        {
            for (int i = obstacleTree_[node].begin_; i < obstacleTree_[node].end_; ++i)
            {

                FixedInt distSq = FixedCalculate.Square(position - obstacles_[i].position_);
                //Find EnemyUnit
                if (distSq < rangeSq)
                {
                    rangeSq = distSq;
                    obstacleNo = obstacles_[i].id_;
                }
            }
        }
        else
        {
            FixedInt distSqLeft = FixedCalculate.Square(FixedCalculate.Max(0, obstacleTree_[obstacleTree_[node].left_].minX_ - position.X)) + FixedCalculate.Square(FixedCalculate.Max(0, position.X - obstacleTree_[obstacleTree_[node].left_].maxX_)) + FixedCalculate.Square(FixedCalculate.Max(0, obstacleTree_[obstacleTree_[node].left_].minY_ - position.Y)) + FixedCalculate.Square(FixedCalculate.Max(0, position.Y - obstacleTree_[obstacleTree_[node].left_].maxY_));
            FixedInt distSqRight = FixedCalculate.Square(FixedCalculate.Max(0, obstacleTree_[obstacleTree_[node].right_].minX_ - position.X)) + FixedCalculate.Square(FixedCalculate.Max(0, position.X - obstacleTree_[obstacleTree_[node].right_].maxX_)) + FixedCalculate.Square(FixedCalculate.Max(0, obstacleTree_[obstacleTree_[node].right_].minY_ - position.Y)) + FixedCalculate.Square(FixedCalculate.Max(0, position.Y - obstacleTree_[obstacleTree_[node].right_].maxY_));

            if (distSqLeft < distSqRight)
            {
                if (distSqLeft < rangeSq)
                {
                    GetClosestObstacle(position, ref rangeSq, ref obstacleNo, obstacleTree_[node].left_);

                    if (distSqRight < rangeSq)
                    {
                        GetClosestObstacle(position, ref rangeSq, ref obstacleNo, obstacleTree_[node].right_);
                    }
                }
            }
            else
            {
                if (distSqRight < rangeSq)
                {
                    GetClosestObstacle(position, ref rangeSq, ref obstacleNo, obstacleTree_[node].right_);

                    if (distSqLeft < rangeSq)
                    {
                        GetClosestObstacle(position, ref rangeSq, ref obstacleNo, obstacleTree_[node].left_);
                    }
                }
            }

        }

    }



}

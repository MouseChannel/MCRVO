using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

// [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[DisableAutoCreation]
public abstract partial class WorkSystem : SystemBase

{
    public abstract void Work();
 
    protected EndSimulationEntityCommandBufferSystem endSimulationEntityCommandBufferSystem;
  
    protected EntityCommandBuffer.ParallelWriter ecbPara
    {
        get
        {
           

            return World.GetExistingSystemManaged<EndSimulationEntityCommandBufferSystem>().CreateCommandBuffer().AsParallelWriter();

        }
    }
    


    protected override void OnCreate()
    {

        endSimulationEntityCommandBufferSystem = World.GetExistingSystemManaged<EndSimulationEntityCommandBufferSystem>();
       
    }
    // public virtual void Init(){}
    protected T GetSystem<T>() where T : SystemBase
    {
        return World.GetOrCreateSystemManaged<T>();
    }



    protected override void OnUpdate()
    {

    }
}


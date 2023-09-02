import { MutableRefObject, useEffect, useRef } from "react";
import { ArrowHelper, BufferAttribute, BufferGeometry, EdgesGeometry, InstancedMesh, LineBasicMaterial, Material, Matrix4, NormalBufferAttributes, Object3D, Vector3 } from "three";

export default function Objects({filteredObjectsRef, count}: {filteredObjectsRef: MutableRefObject<Object[]>, count: number}) {
  let objects = filteredObjectsRef.current.flat()
  objects = objects.length == 0 ? [] : [objects[objects.length-1]]

  const instancedMeshRef = useRef<InstancedMesh<BufferGeometry<NormalBufferAttributes>, Material | Material[]>>()
  const temp = new Object3D()

  let arrowDefaultDirection = new Vector3
  arrowDefaultDirection.set(1,0,0)

  let arrowDefaultLocation = new Vector3
  arrowDefaultLocation.set(0,0,0)

  useEffect(() => {
    objects.forEach(({pos, heading}, i) => {
      temp.position.set(pos[0], pos[2], pos[1]) // y is up in threejs
      let threeRotationMatrix = new Matrix4
      threeRotationMatrix.makeRotationZ(heading)
      temp.setRotationFromMatrix(threeRotationMatrix)
      temp.updateMatrix()
      instancedMeshRef.current!.setMatrixAt(i, temp.matrix)
    })
    instancedMeshRef.current!.instanceMatrix.needsUpdate = true
  }, [count])
  return (
    <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, objects.length]}>
        <arrowHelper args={[arrowDefaultDirection, arrowDefaultLocation, 0.1]}/>
        <meshPhongMaterial/>
    </instancedMesh>
  )
}
import { matrix, multiply, transpose } from "mathjs";
import { BufferAttribute, BufferGeometry, EdgesGeometry, LineBasicMaterial, LineSegments } from "three";

export default function CameraWireframe({R, t, toWorldCoordsMatrix}: {R: Array<Array<number>>, t: Array<number>, toWorldCoordsMatrix: number[][]}) {
  const vertices = [
    [0,0,0],
    [1,0.85,1],
    [-1,0.85,1],
    [-1,-0.85,1],
    [1,-0.85,1]
  ];
  const indices = [0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1];

  let transformationMatrix = matrix([
    R[0].concat(t[0]),
    R[1].concat(t[1]),
    R[2].concat(t[2]),
    [0,0,0,1]
  ])
  const scaleFactor = 0.1
  const scaleMatrix = matrix([
    [scaleFactor,0,0,0],
    [0,scaleFactor,0,0],
    [0,0,scaleFactor,0],
    [0,0,0,1],
  ])
  transformationMatrix = multiply(transformationMatrix, scaleMatrix)
  const flipMatrix = matrix([
    [1,0,0,0],
    [0,-1,0,0],
    [0,0,1,0],
    [0,0,0,1],
  ])
  transformationMatrix = multiply(flipMatrix, transformationMatrix)
  transformationMatrix = multiply(toWorldCoordsMatrix, transformationMatrix)

  const transformedVertices = new Float32Array(
    vertices
      .map(vertex => {
        const point_h = transpose(matrix([vertex.concat([1])]))
        const transformed_h = transpose(multiply(transformationMatrix, point_h))._data[0]
        const transformed = [transformed_h[0]/transformed_h[3], transformed_h[1]/transformed_h[3], transformed_h[2]/transformed_h[3]]
        return transformed
      })
      .flat()
  )

  const geometry = new BufferGeometry();
  geometry.setIndex(indices);
  geometry.setAttribute('position', new BufferAttribute(transformedVertices, 3));

  const wireframeGeo = new EdgesGeometry(geometry);
  const mat = new LineBasicMaterial({color: 0x000000, linewidth: 2});


  return (
    <mesh>
      <lineSegments args={[wireframeGeo, mat]}/>
    </mesh>
  )
}
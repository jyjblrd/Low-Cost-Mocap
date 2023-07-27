import React from 'react';
import dynamic from "next/dynamic";
import Plot from 'react-plotly.js';
import math, { Matrix, matrix, multiply, subtract, transpose } from 'mathjs'

export default function SceneViewer({ cameras }: {cameras: Array<{R: Array<Array<number>>, t: Array<number>}>}) {
  console.log(cameras)
  const unpack = (rows: Array<Array<number>>, key: number) => rows.map(row => row[key])

  const cameraPyramid = [
    [0,0,0],
    [1,1,1],
    [-1,1,1],
    [0,0,0],
    [-1,1,1],
    [-1,-1,1],
    [0,0,0],
    [-1,-1,1],
    [1,-1,1],
    [0,0,0],
    [1,-1,1],
    [1,1,1]
  ]

  const cameraPyramids = cameras.map(({R, t}) => {
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

    const transformedPoints = cameraPyramid.map(point => {
      const point_h = transpose(matrix([point.concat([1])]))
      const transformed_h = transpose(multiply(transformationMatrix, point_h))._data[0]
      const transformed = [transformed_h[0]/transformed_h[3], transformed_h[1]/transformed_h[3], transformed_h[2]/transformed_h[3]]
      return transformed
    })

    return {
      x: unpack(transformedPoints, 0), 
      y: unpack(transformedPoints, 1), 
      z: unpack(transformedPoints, 2),
      mode: 'lines',
      marker: {
        color: 'rgb(127, 127, 127)',
        size: 12,
        symbol: 'circle',
        line: {
        color: 'rgb(204, 204, 204)',
        width: 1},
        opacity: 0.8},
      type: 'scatter3d'}
  })

  const data = cameraPyramids 
  console.log(data)

  return (
    <Plot
      data={data}
      layout={{
        width: window.innerWidth-80, 
        height: window.innerWidth*0.6, 
        title: 'A Fancy Plot',
        autosize: true,
        scene: {
            xaxis: {
                zeroline: false,
                range: [-1,1]
            },
            yaxis: {
                zeroline: false,
                range: [-1,1]
            },
            zaxis: {
                zeroline: false,
                range: [0,2]
            }
        },
      }}
    />
  );
}
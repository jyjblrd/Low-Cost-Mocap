import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  CoreChartOptions,
  DatasetChartOptions,
  ElementChartOptions,
  PluginChartOptions,
  ScaleChartOptions,
  elements,
  LineControllerChartOptions,
  ChartData,
} from 'chart.js';
import { MutableRefObject, forwardRef, useRef } from 'react';
import { Line } from 'react-chartjs-2';
import { text } from 'stream/consumers';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

export const options = {
  animation: false,
  responsive: true,
  spanGaps: true,
  showLine: true,
  plugins: {
    maintainAspectRatio: false,
    tooltip: {
      enabled: false,
    },
    legend: {
      align: 'start',
    },
    title: {
      display: true,
      text: 'Drone Position',
    },
  },
  scales: {
    A: {
      type: 'linear',
      position: 'left',
    }, 
    B: {
      type: 'linear',
      position: 'left',
      max: Math.PI/2,
      min: -Math.PI/2
    }, 
    C: {
      type: 'linear',
      position: 'left'
    }
  },
  elements: {
    point: {
        radius: 0 // default to disabled in all datasets
    }
  }
};

let dataTemplate: ChartData<"line", number[], number> = {
  labels: [] as number[],
  datasets: [
    {
      label: 'X',
      data: [] as number[],
      borderColor: 'rgb(255, 0, 0)',
      backgroundColor: 'rgba(255, 0, 0, 0.5)',
      yAxisID: "A",
    },
    {
      label: 'Y',
      data: [] as number[],
      borderColor: 'rgb(0, 255, 0)',
      backgroundColor: 'rgba(0, 255, 0, 0.5)',
      yAxisID: "A"
    },
    {
      label: 'Z',
      data: [] as number[],
      borderColor: 'rgb(0, 0, 255)',
      backgroundColor: 'rgba(0, 0, 255, 0.5)',
      yAxisID: "A",
    },
    {
      label: 'YAW',
      data: [] as number[],
      borderColor: 'rgb(128, 128, 128)',
      backgroundColor: 'rgba(128, 128, 128, 0.5)',
      yAxisID: "B"
    },
    {
      label: 'X Vel',
      data: [] as number[],
      borderColor: 'rgb(220, 220, 0)',
      backgroundColor: 'rgba(220, 220, 0, 0.5)',
      yAxisID: "C"
    },
    {
      label: 'Y Vel',
      data: [] as number[],
      borderColor: 'rgb(0, 220, 220)',
      backgroundColor: 'rgba(0, 220, 220, 0.5)',
      yAxisID: "C"
    },
    {
      label: 'Z Vel',
      data: [] as number[],
      borderColor: 'rgb(220, 0, 220)',
      backgroundColor: 'rgba(220, 0, 220, 0.5)',
      yAxisID: "C"
    },
    {
      label: 'X Setpoint',
      data: [] as number[],
      borderColor: 'rgb(128, 0, 0)',
      backgroundColor: 'rgba(128, 0, 0, 0.5)',
      yAxisID: "A"
    },
    {
      label: 'Y Setpoint',
      data: [] as number[],
      borderColor: 'rgb(0, 128, 0)',
      backgroundColor: 'rgba(0, 128, 0, 0.5)',
      yAxisID: "A",
    },
    {
      label: 'Z Setpoint',
      data: [] as number[],
      borderColor: 'rgb(0, 0, 128)',
      backgroundColor: 'rgba(0, 0, 128, 0.5)',
      yAxisID: "A"
    },
    {
      label: 'X Vel Setpoint',
      data: [] as number[],
      borderColor: 'rgb(128, 128, 0)',
      backgroundColor: 'rgba(128, 128, 0, 0.5)',
      yAxisID: "C"
    },
    {
      label: 'Y Vel Setpoint',
      data: [] as number[],
      borderColor: 'rgb(0, 128, 128)',
      backgroundColor: 'rgba(0, 128, 128, 0.5)',
      yAxisID: "C"
    },
    {
      label: 'Z Vel Setpoint',
      data: [] as number[],
      borderColor: 'rgb(128, 0, 128)',
      backgroundColor: 'rgba(128, 0, 128, 0.5)',
      yAxisID: "C"
    },
  ],
};

let data = structuredClone(dataTemplate)

export default function Chart({filteredObjectsRef, droneSetpointHistoryRef, xyPosKp, zPosKp}: 
  {filteredObjectsRef: MutableRefObject<object>, droneSetpointHistoryRef: MutableRefObject<number[][]>, xyPosKp: number, zPosKp: number}) {
  let sliced = filteredObjectsRef.current.length <= 15 ? [] : filteredObjectsRef.current.slice(15)
  let chartRef = useRef<ChartJS<"line", number[], number> | null>(null);

  const length = sliced.length
  
  if (length === 0) {
    data = structuredClone(dataTemplate)
  }
  else if (length !== data.labels![data.labels!.length - 1]) {
    data.labels.push(length)
    const lastFilteredPoint = sliced[length-1]

    data.datasets[0].data.push(lastFilteredPoint["pos"][0])
    data.datasets[1].data.push(lastFilteredPoint["pos"][1])
    data.datasets[2].data.push(lastFilteredPoint["pos"][2])

    data.datasets[3].data.push(lastFilteredPoint["heading"])
    
    if (lastFilteredPoint["vel"]) {
      data.datasets[4].data.push(lastFilteredPoint["vel"][0])
      data.datasets[5].data.push(lastFilteredPoint["vel"][1])
      data.datasets[6].data.push(lastFilteredPoint["vel"][2])
    }
    else {
      data.datasets[4].data.push(undefined)
      data.datasets[5].data.push(undefined)
      data.datasets[6].data.push(undefined)
    }
    
    data.datasets[7].data.push(droneSetpointHistoryRef.current[length-1][0])
    data.datasets[8].data.push(droneSetpointHistoryRef.current[length-1][1])
    data.datasets[9].data.push(droneSetpointHistoryRef.current[length-1][2])
    
    data.datasets[10].data.push(xyPosKp * (droneSetpointHistoryRef.current[length-1][0] - lastFilteredPoint["pos"][0]))
    data.datasets[11].data.push(xyPosKp * (droneSetpointHistoryRef.current[length-1][1] - lastFilteredPoint["pos"][1]))
    data.datasets[12].data.push(zPosKp * (droneSetpointHistoryRef.current[length-1][2] - lastFilteredPoint["pos"][2]))
  }
  
  chartRef.current?.update()

  return <Line ref={chartRef} options={options} data={data} height={"50px"}/>;
}

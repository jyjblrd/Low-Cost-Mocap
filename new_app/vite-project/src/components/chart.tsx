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
} from 'chart.js';
import { MutableRefObject } from 'react';
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
    yAxes: [
        {
        id: 'A',
        type: 'linear',
        position: 'left',
      }, 
      {
        id: 'B',
        type: 'linear',
        position: 'left',
        ticks: {
          max: Math.PI/2,
          min: -Math.PI/2
        }
      }, 
      {
        id: 'C',
        type: 'linear',
        position: 'left'
      }
    ]
  },
  elements: {
    point: {
        radius: 0 // default to disabled in all datasets
    }
  }
};

export default function Chart({filteredPointsRef}: {filteredPointsRef: MutableRefObject<object>}) {
  let sliced = filteredPointsRef.current.length <= 15 ? [] : filteredPointsRef.current.slice(15)

  const data = {
    labels: sliced.map((_, i) => i),
    datasets: [
      {
        label: 'X',
        data: sliced.map((filteredPoints) => filteredPoints["pos"][0]),
        borderColor: 'rgb(255, 0, 0)',
        backgroundColor: 'rgba(255, 0, 0, 0.5)',
        yAxisID: "A"
      },
      {
        label: 'Y',
        data: sliced.map((filteredPoints) => filteredPoints["pos"][1]),
        borderColor: 'rgb(0, 255, 0)',
        backgroundColor: 'rgba(0, 255, 0, 0.5)',
        yAxisID: "A"
      },
      {
        label: 'Z',
        data: sliced.map((filteredPoints) => filteredPoints["pos"][2]),
        borderColor: 'rgb(0, 0, 255)',
        backgroundColor: 'rgba(0, 0, 255, 0.5)',
        yAxisID: "A"
      },
      {
        label: 'YAW',
        data: sliced.map((filteredPoints) => filteredPoints["heading"]),
        borderColor: 'rgb(128, 128, 128)',
        backgroundColor: 'rgba(128, 128, 128, 0.5)',
        yAxisID: "B"
      },
      {
        label: 'X Vel',
        data: sliced.map((filteredPoints) => filteredPoints["vel"] ? filteredPoints["vel"][0] : undefined),
        borderColor: 'rgb(220, 220, 0)',
        backgroundColor: 'rgba(220, 220, 0, 0.5)',
        yAxisID: "C"
      },
      {
        label: 'Y Vel',
        data: sliced.map((filteredPoints) => filteredPoints["vel"] ? filteredPoints["vel"][1] : undefined),
        borderColor: 'rgb(0, 220, 220)',
        backgroundColor: 'rgba(0, 220, 220, 0.5)',
        yAxisID: "C"
      },
      {
        label: 'Z Vel',
        data: sliced.map((filteredPoints) => filteredPoints["vel"] ? filteredPoints["vel"][2] : undefined),
        borderColor: 'rgb(220, 0, 220)',
        backgroundColor: 'rgba(220, 0, 220, 0.5)',
        yAxisID: "C"
      },
    ],
  };
  return <Line options={options} data={data} height={"50px"} />;
}

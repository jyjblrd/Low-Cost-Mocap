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
    yAxes: [{
      id: 'A',
      type: 'linear',
      position: 'left',
    }, {
      id: 'B',
      type: 'linear',
      position: 'left',
      ticks: {
        max: Math.PI/2,
        min: -Math.PI/2
      }
    }]
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
        borderColor: 'rgb(255, 0, 255)',
        backgroundColor: 'rgba(255, 0, 255, 0.5)',
        yAxisID: "B"
      },
    ],
  };
  return <Line options={options} data={data} height={"50px"} />;
}

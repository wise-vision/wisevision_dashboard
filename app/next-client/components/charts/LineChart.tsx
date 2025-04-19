'use client';

import { useRef, useEffect } from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  ChartData,
  ChartOptions
} from 'chart.js';
import { Line } from 'react-chartjs-2';
import { FullDateTime } from '../../types/ros2-types';

// Register Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

interface LineChartProps {
  title: string;
  data: number[];
  timestamps: FullDateTime[];
  label?: string;
  color?: string;
  fill?: boolean;
  height?: number;
  yAxisLabel?: string;
  xAxisLabel?: string;
  minY?: number;
  maxY?: number;
}

const LineChart: React.FC<LineChartProps> = ({
  title,
  data,
  timestamps,
  label = 'Value',
  color = '#0073ff',
  fill = false,
  height = 300,
  yAxisLabel = '',
  xAxisLabel = '',
  minY,
  maxY
}) => {
  const chartRef = useRef<ChartJS>(null);

  // Format timestamps for display
  const formatTime = (timestamp: FullDateTime) => {
    return `${timestamp.hour.toString().padStart(2, '0')}:${timestamp.minute.toString().padStart(2, '0')}:${timestamp.second.toString().padStart(2, '0')}`;
  };

  // Format full date for tooltip
  const formatFullDate = (timestamp: FullDateTime) => {
    return `${timestamp.year}-${timestamp.month.toString().padStart(2, '0')}-${timestamp.day.toString().padStart(2, '0')} ${formatTime(timestamp)}`;
  };

  const chartData: ChartData<'line'> = {
    labels: timestamps.map(formatTime),
    datasets: [
      {
        label,
        data,
        borderColor: color,
        backgroundColor: fill ? `${color}20` : 'transparent',
        borderWidth: 2,
        pointBackgroundColor: color,
        pointBorderColor: '#fff',
        pointRadius: 4,
        pointHoverRadius: 6,
        fill,
        tension: 0.2,
      },
    ],
  };

  const options: ChartOptions<'line'> = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top',
        labels: {
          usePointStyle: true,
          boxWidth: 6,
        },
      },
      title: {
        display: !!title,
        text: title,
        font: {
          size: 16,
        },
      },
      tooltip: {
        callbacks: {
          title: (items) => {
            const index = items[0].dataIndex;
            return formatFullDate(timestamps[index]);
          }
        }
      }
    },
    scales: {
      y: {
        title: {
          display: !!yAxisLabel,
          text: yAxisLabel,
        },
        min: minY,
        max: maxY,
        ticks: {
          // Add padding so that the axis label isn't on top of the data points
          padding: 5,
        }
      },
      x: {
        title: {
          display: !!xAxisLabel,
          text: xAxisLabel,
        },
        ticks: {
          maxRotation: 45,
          minRotation: 0,
        }
      }
    },
    interaction: {
      intersect: false,
      mode: 'index',
    },
  };

  return (
    <div style={{ height: `${height}px` }}>
      <Line ref={chartRef} data={chartData} options={options} />
    </div>
  );
};

export default LineChart;
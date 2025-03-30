/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState } from 'react';
import { Responsive, WidthProvider } from 'react-grid-layout';
import PropTypes from 'prop-types';
import Card from './ui/Card';
import LineChart from './charts/LineChart';
import GpsChart from './charts/GpsChart';
import PieChart from './charts/PieChart';
import './DashboardGrid.css';

const ResponsiveGridLayout = WidthProvider(Responsive);

// Default layout configuration for different screen sizes
const DEFAULT_LAYOUTS = {
  lg: [],
  md: [],
  sm: [],
  xs: [],
  xxs: []
};

const DashboardGrid = ({ 
  charts, 
  layoutConfig, 
  updateLayoutConfig,
  isDarkMode = false 
}) => {
  const [activeDrag, setActiveDrag] = useState(null);

  // Generate layout from charts if no layout config is provided
  const generateLayoutFromCharts = () => {
    if (!charts || charts.length === 0) return DEFAULT_LAYOUTS;
    
    // Basic layout generation - position charts in a grid
    const layouts = { ...DEFAULT_LAYOUTS };
    
    charts.forEach((chart, index) => {
      // Calculate grid positions (2 columns layout)
      const row = Math.floor(index / 2);
      const col = index % 2;
      
      // Create layout item for each chart
      const layoutItem = {
        i: chart.id || `chart-${index}`,
        x: col * 6, // 6 columns per chart (12 column grid total)
        y: row * 8, // Each chart takes 8 rows
        w: 6,       // Width of 6 columns (half of 12-column grid)
        h: 8,       // Height of 8 rows
        minW: 3,    // Minimum width
        minH: 4     // Minimum height
      };
      
      // Add to all breakpoints
      layouts.lg.push(layoutItem);
      
      // Adjust for smaller screens
      layouts.md.push({ ...layoutItem, w: 6 });
      layouts.sm.push({ ...layoutItem, x: 0, w: 12 }); // Full width on small screens
      layouts.xs.push({ ...layoutItem, x: 0, w: 12 });
      layouts.xxs.push({ ...layoutItem, x: 0, w: 12 });
    });
    
    return layouts;
  };

  // Use provided layout config or generate from charts
  const layouts = layoutConfig || generateLayoutFromCharts();

  // Handle layout changes
  const handleLayoutChange = (currentLayout, allLayouts) => {
    updateLayoutConfig(allLayouts);
  };

  // Render different chart types based on chart.type
  const renderChart = (chart) => {
    switch (chart.type) {
      case 'line':
        return <LineChart 
          topic={chart.topic} 
          label={chart.label}
          isDarkMode={isDarkMode}
        />;
      case 'gps':
        return <GpsChart 
          isDarkMode={isDarkMode}
        />;
      case 'pie':
        return <PieChart 
          data={chart.data} 
          label={chart.label}
          isDarkMode={isDarkMode}
        />;
      default:
        return <div className="chart-error">Unknown chart type: {chart.type}</div>;
    }
  };

  return (
    <div className={`dashboard-grid ${isDarkMode ? 'dashboard-grid-dark' : ''}`}>
      {charts.length > 0 ? (
        <ResponsiveGridLayout
          className="layout"
          layouts={layouts}
          breakpoints={{ lg: 1200, md: 996, sm: 768, xs: 480, xxs: 0 }}
          cols={{ lg: 12, md: 12, sm: 12, xs: 12, xxs: 12 }}
          rowHeight={30}
          margin={[16, 16]}
          onLayoutChange={handleLayoutChange}
          onDragStart={(layout, oldItem, newItem, placeholder, e, element) => {
            setActiveDrag(newItem.i);
          }}
          onDragStop={() => {
            setActiveDrag(null);
          }}
          draggableHandle=".card-header"
          isBounded={true}
          useCSSTransforms={true}
          compactType="vertical"
          preventCollision={false}
          isResizable={true}
          resizeHandles={['se']}
          autoSize={true}
          verticalCompact={true}
        >
          {charts.map((chart, index) => {
            const chartId = chart.id || `chart-${index}`;
            return (
              <div key={chartId} className={activeDrag === chartId ? 'dragging' : ''}>
                <Card
                  title={chart.label || `Chart ${index + 1}`}
                  elevation="md"
                  className={`chart-card ${isDarkMode ? 'chart-card-dark' : ''}`}
                  actions={
                    <div className="chart-actions">
                      {/* Add chart action buttons here if needed */}
                    </div>
                  }
                >
                  {renderChart(chart)}
                </Card>
              </div>
            );
          })}
        </ResponsiveGridLayout>
      ) : (
        <div className="no-charts">
          <h3>No charts to display</h3>
          <p>Add charts using the sidebar to get started.</p>
        </div>
      )}
    </div>
  );
};

DashboardGrid.propTypes = {
  charts: PropTypes.array.isRequired,
  layoutConfig: PropTypes.object,
  updateLayoutConfig: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool
};

export default DashboardGrid;

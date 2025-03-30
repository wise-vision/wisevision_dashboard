/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

const LAYOUT_STORAGE_KEY = 'wisevision_dashboard_layout';
const CHARTS_STORAGE_KEY = 'charts';

export class LayoutManager {
  /**
   * Saves the current dashboard layout to localStorage
   * @param {Array} layout - The layout configuration
   * @param {Array} charts - The charts data
   */
  static saveLayout(layout, charts) {
    try {
      const layoutData = {
        layout,
        lastUpdated: new Date().toISOString(),
      };
      
      localStorage.setItem(LAYOUT_STORAGE_KEY, JSON.stringify(layoutData));
      localStorage.setItem(CHARTS_STORAGE_KEY, JSON.stringify(charts));
      
      return true;
    } catch (error) {
      console.error('Error saving layout:', error);
      return false;
    }
  }

  /**
   * Loads the saved dashboard layout from localStorage
   * @returns {Object|null} The saved layout or null if not found
   */
  static loadLayout() {
    try {
      const layoutData = localStorage.getItem(LAYOUT_STORAGE_KEY);
      if (!layoutData) return null;
      
      return JSON.parse(layoutData);
    } catch (error) {
      console.error('Error loading layout:', error);
      return null;
    }
  }

  /**
   * Loads the saved charts from localStorage
   * @returns {Array} The saved charts or empty array if not found
   */
  static loadCharts() {
    try {
      const charts = localStorage.getItem(CHARTS_STORAGE_KEY);
      return charts ? JSON.parse(charts) : [];
    } catch (error) {
      console.error('Error loading charts:', error);
      return [];
    }
  }

  /**
   * Updates a chart in the layout
   * @param {string} chartId - ID of the chart to update
   * @param {Object} updatedData - Updated chart data
   * @param {Array} currentCharts - Current charts array
   * @returns {Array} Updated charts array
   */
  static updateChart(chartId, updatedData, currentCharts) {
    const updatedCharts = currentCharts.map(chart => 
      chart.id === chartId ? { ...chart, ...updatedData } : chart
    );
    
    localStorage.setItem(CHARTS_STORAGE_KEY, JSON.stringify(updatedCharts));
    return updatedCharts;
  }

  /**
   * Adds a new chart to the layout
   * @param {Object} newChart - New chart data
   * @param {Array} currentCharts - Current charts array
   * @returns {Array} Updated charts array
   */
  static addChart(newChart, currentCharts) {
    const updatedCharts = [...currentCharts, newChart];
    localStorage.setItem(CHARTS_STORAGE_KEY, JSON.stringify(updatedCharts));
    return updatedCharts;
  }

  /**
   * Removes a chart from the layout
   * @param {string} chartId - ID of the chart to remove
   * @param {Array} currentCharts - Current charts array
   * @returns {Array} Updated charts array
   */
  static removeChart(chartId, currentCharts) {
    const updatedCharts = currentCharts.filter(chart => chart.id !== chartId);
    localStorage.setItem(CHARTS_STORAGE_KEY, JSON.stringify(updatedCharts));
    return updatedCharts;
  }

  /**
   * Clear stored layout and charts
   */
  static resetLayout() {
    localStorage.removeItem(LAYOUT_STORAGE_KEY);
    localStorage.removeItem(CHARTS_STORAGE_KEY);
  }
}

export default LayoutManager;

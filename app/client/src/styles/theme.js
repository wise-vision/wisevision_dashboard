/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

export const theme = {
  colors: {
    // Primary palette
    primary: {
      100: '#E3F2FD', // Lightest blue
      200: '#BBDEFB',
      300: '#90CAF9',
      400: '#64B5F6',
      500: '#42A5F5', // Main brand blue
      600: '#2196F3',
      700: '#1E88E5',
      800: '#1565C0',
      900: '#0D47A1', // Darkest blue
    },
    
    // Secondary accent
    accent: {
      main: '#4BCDF0',
      dark: '#20428B',
    },
    
    // Semantic colors
    success: '#4CAF50',
    warning: '#FFC107',
    error: '#F44336',
    info: '#2196F3',
    
    // Grayscale
    gray: {
      50: '#FAFAFA',
      100: '#F5F5F5',
      200: '#EEEEEE',
      300: '#E0E0E0',
      400: '#BDBDBD',
      500: '#9E9E9E',
      600: '#757575',
      700: '#616161',
      800: '#424242',
      900: '#212121',
    },
    
    // Base colors
    background: {
      default: '#F4F5F6',
      paper: '#FFFFFF',
      card: '#FFFFFF',
      dark: '#1A1F36',
    },
    text: {
      primary: '#263238',
      secondary: '#546E7A',
      disabled: '#9E9E9E',
      light: '#FFFFFF',
    },
    divider: '#E0E0E0',
  },
  
  typography: {
    fontFamily: "'Roboto Condensed', sans-serif",
    fontWeights: {
      light: 300,
      regular: 400,
      medium: 500,
      bold: 700,
    },
    fontSize: {
      xs: '0.75rem',    // 12px
      sm: '0.875rem',   // 14px
      md: '1rem',       // 16px
      lg: '1.125rem',   // 18px
      xl: '1.25rem',    // 20px
      '2xl': '1.5rem',  // 24px
      '3xl': '1.875rem', // 30px
      '4xl': '2.25rem',  // 36px
    },
  },
  
  spacing: {
    xs: '0.25rem',  // 4px
    sm: '0.5rem',   // 8px
    md: '1rem',     // 16px
    lg: '1.5rem',   // 24px
    xl: '2rem',     // 32px
    '2xl': '2.5rem', // 40px
    '3xl': '3rem',   // 48px
  },
  
  shadows: {
    sm: '0 1px 3px rgba(0,0,0,0.12), 0 1px 2px rgba(0,0,0,0.14)',
    md: '0 4px 6px rgba(0,0,0,0.12), 0 1px 3px rgba(0,0,0,0.08)',
    lg: '0 10px 15px rgba(0,0,0,0.12), 0 4px 6px rgba(0,0,0,0.08)',
    xl: '0 20px 25px rgba(0,0,0,0.12), 0 10px 10px rgba(0,0,0,0.08)',
  },
  
  animation: {
    fast: '0.15s ease-in-out',
    normal: '0.3s ease-in-out',
    slow: '0.5s ease-in-out',
  },
  
  borderRadius: {
    sm: '4px',
    md: '8px',
    lg: '12px',
    xl: '16px',
    '2xl': '20px',
    full: '9999px',
  },
};

export default theme;

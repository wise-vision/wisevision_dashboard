import { render, screen } from '@testing-library/react';
import { ThemeProvider, useTheme } from './theme-provider';
import React from 'react';

describe('ThemeProvider', () => {
  it('provides the default theme', () => {
    const TestComponent = () => {
      const { theme } = useTheme();
      return <div>{theme}</div>;
    };

    render(
      <ThemeProvider defaultTheme="dark">
        <TestComponent />
      </ThemeProvider>
    );

    expect(screen.getByText('dark')).toBeInTheDocument();
  });

  it('updates the theme and persists it in localStorage', () => {
    const TestComponent = () => {
      const { theme, setTheme } = useTheme();
      return (
        <div>
          <span>{theme}</span>
          <button onClick={() => setTheme('light')}>Set Light Theme</button>
        </div>
      );
    };

    render(
      <ThemeProvider defaultTheme="dark">
        <TestComponent />
      </ThemeProvider>
    );

    expect(screen.getByText('dark')).toBeInTheDocument();

    screen.getByText('Set Light Theme').click();

    expect(screen.getByText('light')).toBeInTheDocument();
    expect(localStorage.getItem('theme')).toBe('light');
  });
});
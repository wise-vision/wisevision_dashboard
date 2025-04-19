import { render, screen, fireEvent } from '@testing-library/react';
import { ThemeProvider } from '../../../components/theme-provider';
import SettingsPage from './page';

describe('SettingsPage', () => {
  it('renders and toggles the theme', () => {
    render(
      <ThemeProvider defaultTheme="light">
        <SettingsPage />
      </ThemeProvider>
    );

    // Verify initial theme is light
    expect(document.documentElement.getAttribute('data-theme')).toBe('light');

    // Toggle to dark theme
    const darkModeToggle = screen.getByLabelText('Dark Mode');
    fireEvent.click(darkModeToggle);

    // Verify theme is updated to dark
    expect(document.documentElement.getAttribute('data-theme')).toBe('dark');
  });

  it('saves settings and shows feedback', () => {
    render(
      <ThemeProvider defaultTheme="light">
        <SettingsPage />
      </ThemeProvider>
    );

    // Click save button
    const saveButton = screen.getByText('Save Settings');
    fireEvent.click(saveButton);

    // Verify saving feedback
    expect(screen.getByText('Saving...')).toBeInTheDocument();

    // Wait for saved feedback
    setTimeout(() => {
      expect(screen.getByText('Saved!')).toBeInTheDocument();
    }, 1000);
  });
});
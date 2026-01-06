import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import LanguageSwitcher from '../index';

// Mock the Docusaurus context and router hooks
jest.mock('@docusaurus/core/lib/client/exports/contexts', () => ({
  useLocaleContext: jest.fn(() => ({ locale: 'en', setLocale: jest.fn() })),
}));

jest.mock('@docusaurus/router', () => ({
  useLocation: jest.fn(() => ({ pathname: '/en/docs/intro' })),
  useNavigate: jest.fn(() => jest.fn()),
}));

// Mock the translate function
jest.mock('@docusaurus/Translate', () => ({
  translate: jest.fn((obj) => obj.message || 'Language switcher'),
}));

describe('LanguageSwitcher', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders with current language', () => {
    render(<LanguageSwitcher />);
    
    // Check that the current language is displayed
    expect(screen.getByText('English')).toBeInTheDocument();
  });

  test('toggles dropdown when clicked', () => {
    render(<LanguageSwitcher />);
    
    // Initially, the dropdown should not be visible
    expect(screen.queryByText('اردو')).not.toBeInTheDocument();
    
    // Click the language switcher button
    fireEvent.click(screen.getByLabelText('Language switcher'));
    
    // Now the dropdown should be visible with Urdu option
    expect(screen.getByText('اردو')).toBeInTheDocument();
  });

  test('changes language when option is selected', () => {
    const setLocaleMock = jest.fn();
    const navigateMock = jest.fn();
    
    // Mock the hooks to return our test functions
    const mockUseLocaleContext = require('@docusaurus/core/lib/client/exports/contexts');
    mockUseLocaleContext.useLocaleContext.mockReturnValue({
      locale: 'en',
      setLocale: setLocaleMock,
    });
    
    const mockUseRouter = require('@docusaurus/router');
    mockUseRouter.useNavigate.mockReturnValue(navigateMock);
    
    render(<LanguageSwitcher />);
    
    // Click the language switcher button to open dropdown
    fireEvent.click(screen.getByLabelText('Language switcher'));
    
    // Click on the Urdu option
    fireEvent.click(screen.getByText('اردو'));
    
    // Verify that locale was updated and navigation occurred
    expect(setLocaleMock).toHaveBeenCalledWith('ur');
    expect(navigateMock).toHaveBeenCalledWith('/ur/docs/intro', { replace: true });
  });
});
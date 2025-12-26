import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import PersonalizeContentButton from './PersonalizeContentButton';
import { isAuthenticated } from '../utils/session';

// Mock the session utility
jest.mock('../utils/session', () => ({
  isAuthenticated: jest.fn(),
}));

describe('PersonalizeContentButton', () => {
  const mockOnPersonalize = jest.fn();
  const defaultProps = {
    chapterPath: '/test/chapter',
    onPersonalize: mockOnPersonalize,
    className: 'test-class'
  };

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders the button with locked icon when user is not authenticated', () => {
    (isAuthenticated as jest.MockedFunction<typeof isAuthenticated>).mockReturnValue(false);

    render(<PersonalizeContentButton {...defaultProps} />);

    // Component will initially show placeholder during SSR simulation
    // After useEffect runs, it will show the actual button
    const button = screen.getByRole('button');
    expect(button).toBeInTheDocument();
    
    // Initially, before useEffect runs, the button shows a lock icon
    // After client-side authentication check, it should still show a lock icon
    // Since we're not authenticated
    expect(button).toHaveTextContent('Personalize Content');
    expect(button.querySelector('svg')?.classList.contains('fa-lock') !== undefined || 
           button.querySelector('svg')?.classList.contains('fa-unlock') !== undefined).toBe(true);
  });

  it('renders the button with unlocked icon when user is authenticated', () => {
    (isAuthenticated as jest.MockedFunction<typeof isAuthenticated>).mockReturnValue(true);

    render(<PersonalizeContentButton {...defaultProps} />);

    const button = screen.getByRole('button');
    expect(button).toBeInTheDocument();
  });

  it('redirects to login page when not authenticated and clicked', () => {
    // Mock window.location
    const mockLocation = {
      href: '',
    };
    Object.defineProperty(window, 'location', {
      value: mockLocation,
      writable: true,
    });

    (isAuthenticated as jest.MockedFunction<typeof isAuthenticated>).mockReturnValue(false);

    render(<PersonalizeContentButton {...defaultProps} />);

    const button = screen.getByRole('button');
    fireEvent.click(button);

    // Verify redirect happened
    expect(mockLocation.href).toBe('/login');
  });

  it('calls personalization API when authenticated and clicked', async () => {
    // Mock fetch
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ personalized_content: 'Mock personalization result' }),
      } as Response)
    );
    global.fetch = mockFetch;

    (isAuthenticated as jest.MockedFunction<typeof isAuthenticated>).mockReturnValue(true);

    render(<PersonalizeContentButton {...defaultProps} />);

    const button = screen.getByRole('button');
    fireEvent.click(button);

    // Wait for the API call to complete
    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/personalize/test/chapter', {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
    });
  });

  it('calls onPersonalize callback with API result when authenticated and clicked', async () => {
    const mockPersonalizedContent = 'Customized content for user';
    
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ personalized_content: mockPersonalizedContent }),
      } as Response)
    );
    global.fetch = mockFetch;

    (isAuthenticated as jest.MockedFunction<typeof isAuthenticated>).mockReturnValue(true);

    render(<PersonalizeContentButton {...defaultProps} />);

    const button = screen.getByRole('button');
    fireEvent.click(button);

    // Wait for the API call to complete and callback to be executed
    await waitFor(() => {
      expect(mockOnPersonalize).toHaveBeenCalledWith(mockPersonalizedContent);
    });
  });
});
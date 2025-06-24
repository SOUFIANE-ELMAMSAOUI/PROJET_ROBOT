import React from 'react';
import { render, screen } from '@testing-library/react';
import RobotProjectSite from './App';

test('renders robot project site', () => {
  render(<RobotProjectSite />);
  const linkElement = screen.getByText(/Robot Autonome/i);
  expect(linkElement).toBeInTheDocument();
});
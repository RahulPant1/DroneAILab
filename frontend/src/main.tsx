import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import App from './App';
import { sim } from './sim';
import './styles.css';

sim.connect();

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);

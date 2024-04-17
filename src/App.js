import React, { Suspense, lazy } from 'react';
import {
  BrowserRouter as Router,
  Routes, // Use Routes instead of Switch
  Route,
} from 'react-router-dom';

// Lazy imports for your components
const Home = lazy(() => import('./components/Home'));
const TestFront = lazy(() => import('./modules/TestFront'));
const AnotherFront = lazy(() => import('./modules/Test2Front'));

function App() {
  return (
    <Router>
      <Suspense fallback={<div>Loading...</div>}>
        <Routes>
          <Route path="/" element={<Home />} /> {/* Home component now wrapped in element prop */}
          <Route path="/testfront" element={<TestFront />} />
          <Route path="/anotherfront" element={<AnotherFront />} />
          {/* Add more routes as needed */}
        </Routes>
      </Suspense>
    </Router>
  );
}

export default App;
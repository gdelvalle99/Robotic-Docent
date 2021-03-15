import logo from './logo.svg';
import React from 'react';
import './App.css';
import QAGenerator from './questionare'

function App() {
  return (
    <div className="App">
      <heading>Robot Docent Mobile</heading>
      <header className="App-header">
        <QAGenerator />     
      </header>
    </div>
  );
}

export default App;

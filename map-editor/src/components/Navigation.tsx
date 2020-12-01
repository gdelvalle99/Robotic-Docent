import React, { useState, ReactEventHandler } from "react";
import { BrowserRouter as Router, Route, Link } from "react-router-dom";
import { Button } from '@material-ui/core';
import Home from '../pages/Home';
import Analytics from '../pages/Analytics';
import { Editor } from '../pages/Editor';

export const NavigationBar: React.SFC = (props) => {
    return (
        <Router>
            <div className="navigation-container">
                <nav className="navigation-bar">
                    <div className="navigation-bar-title-container">
                        <a href="/">
                            <h1 className="navigation-bar-title">
                                Robotic Docent
                                <span className="navigation-bar-title-version">1.0</span>
                            </h1>
                        </a>
                    </div>
                    <div className="navigation-button-container">
                        <Button className="navigation-button" variant="contained" href="/">Home</Button>
                        <Button className="navigation-button" variant="contained" href="/map-editor">Editor</Button>
                        <Button className="navigation-button" variant="contained" href="/analytics">Analytics</Button>
                    </div>
                </nav>

                <Route path="/" exact component={Home} />
                <Route path="/map-editor" exact component={Editor} />
                <Route path="/analytics" exact component={Analytics} />

            </div>
        </Router>
    );
};
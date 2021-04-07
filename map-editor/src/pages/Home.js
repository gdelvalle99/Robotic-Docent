import React, { useState } from "react";
import { Card } from "@material-ui/core";
import { CardHeader } from "@material-ui/core";
import { CardContent } from "@material-ui/core";
import { CardMedia } from "@material-ui/core";
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import MapImage from "../assets/images/isometric_stock_map.jpg";
import axios from 'axios'

const userActivity = [
    {
        name: "Guillermo Del Valle",
        activity: "17 seconds ago"
    },
    {
        name: "Matthew Alighchi",
        activity: "34 minutes ago"
    },
    {
        name: "Kyle Respicio",
        activity: "3/14/21 4:23 PM"
    },
    {
        name: "Sherman Lee",
        activity: "1/23/21 8:20 AM"
    }

]

export const Home = () => {

    return (
        <div className="home-page">
            <div className="home-map-and-profile">
                    <div className="home-map-card">
                        <a href="/map-editor">
                            <Card>
                                <CardHeader title="Map Editor" subheader="Edit your museum"/>
                                <CardMedia image={MapImage}/>
                            </Card>
                        </a>
                    </div>
                <div className="home-profile-card">
                    <Card>
                        <CardHeader title="Log History"/>
                        <CardContent className="home-profile-list">
                            {userActivity.map(user => {
                                return (
                                    <div className="home-profile">
                                        <AccountCircleIcon fontSize="large"/>
                                        <div className="home-profile-info-container">
                                            <div className="home-profile-name">
                                                {user.name}
                                            </div>
                                            <div className="home-profile-activity">
                                                Last Activity: {user.activity}
                                            </div>
                                        </div>
                                    </div>    
                                )
                            })
                            }
                        </CardContent>
                    </Card>
                </div>
            </div>
            <div className="home-analytics-card">
                <Card>
                    <CardHeader title="Today's Analytics"/>
                </Card>
            </div>
        </div>
    );
}
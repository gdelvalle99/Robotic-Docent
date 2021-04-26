import React, { useState, useEffect } from "react";

import { makeStyles } from "@material-ui/core/styles";
import { Card } from '@material-ui/core';
import { CardHeader } from '@material-ui/core'; 
import { TourModalPiece } from './TourModalPiece';
import Typography from "@material-ui/core/Typography";


const useStyles = makeStyles((theme) => ({
    text: {
        padding: theme.spacing(2, 2, 0),
    },
}));

export const TourList = (props) => {
    const classes = useStyles();

    const [tours,setTours] = useState([{title: "Tour 1", id: "437829174fdios", subtitle: "this is first tour", date: "Mon Apr 26 2021 14:41:56 GMT-0700"},
    {title: "Tour 2", id: "437829174fdios", subtitle: "this is first tour", date: "Mon Apr 27 2021 14:41:56 GMT-0700"},
    {title: "Tour 3", id: "437829174fdios", subtitle: "this is first tour", date: "Mon Apr 25 2021 14:41:56 GMT-0700"},
    {title: "Tour 4", id: "437829174fdios", subtitle: "this is first tour", date: "Mon Apr 22 2021 14:41:56 GMT-0700"},
    {title: "Tour 3", id: "437829174fdios", subtitle: "this is first tour", date: "Mon Apr 28 2021 14:41:56 GMT-0700"}])

    const [prev, setPrev] = useState([]);
    const [future, setFuture] = useState([]);
    const [today, setToday] = useState([])


    useEffect(() => {
        const today = new Date()
        let previous = tours.filter(x=>new Date(x.date) < today);
        setPrev(previous)

        let fut = tours.filter(x=>new Date(x.date) > today);

        setFuture(fut)

        

        const isToday = (someDate) => {
            const today = new Date()
            return someDate.getDate() == today.getDate() &&
              someDate.getMonth() == today.getMonth() &&
              someDate.getFullYear() == today.getFullYear()
          }

        let t = tours.filter(x=>isToday(new Date(x.date)));
        setToday(t);

    }, [])

    return (
        <div>
            <Typography className={classes.text} variant="h5" gutterBottom>
            Today's Tours
            </Typography>
            {today.map(t => <TourModalPiece tour={t}/>)}
            <Typography className={classes.text} variant="h5" gutterBottom>
            Upcoming Tours
            {future.map(t => <TourModalPiece tour={t}/>)}
            </Typography>
            <Typography className={classes.text} variant="h5" gutterBottom>
            Past Tours
            {prev.map(t => <TourModalPiece tour={t}/>)}
            </Typography>
            
        </div>
    );
};

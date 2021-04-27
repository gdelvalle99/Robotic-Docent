import React, { useState, useEffect } from "react";
import { TourModalPiece } from './TourModalPiece';
import Typography from "@material-ui/core/Typography";


export const TourList = ({tourList}) => {
    const [prev, setPrev] = useState([]);
    const [future, setFuture] = useState([]);
    const [today, setToday] = useState([])

    useEffect(() => {
        // console.log(tourList);
        const today = new Date()
        let previous = tourList.filter(x=>new Date(x.start_date) < today);
        setPrev(previous)

        let fut = tourList.filter(x=>new Date(x.start_date) > today);

        setFuture(fut)        

        const isToday = (someDate) => {
            const today = new Date()
            return someDate.getDate() == today.getDate() &&
              someDate.getMonth() == today.getMonth() &&
              someDate.getFullYear() == today.getFullYear()
          }

        let t = tourList.filter(x=>isToday(new Date(x.start_date)));
        setToday(t);

    }, [tourList]);

    return (
        <div className="tour-list-container">
            <div className="tour-list-today">
                <Typography variant="h5" gutterBottom>
                Today's Tours
                </Typography>
                {today.map(t => <TourModalPiece key={t.id} tour={t}/>)}
            </div>
            <div className="tour-list-upcoming">
                <Typography  variant="h5" gutterBottom>
                Upcoming Tours
                </Typography>
                {future.map(t => <TourModalPiece key={t.id} tour={t}/>)}
            </div>
            <div className="tour-list-past">
                <Typography variant="h5" gutterBottom>
                Past Tours
                </Typography>
                {prev.map(t => <TourModalPiece key={t.id} tour={t}/>)}
            </div>            
        </div>
    );
};

import React, { useState, useEffect } from "react";
import { TourList } from '../components/TourList';
import { floorLink } from "../links";
import axios from 'axios';

export const Tours = () => {

    return (
        <div className="tours-page">
            <TourList/>
        </div>
    );

}
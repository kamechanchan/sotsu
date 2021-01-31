#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import math


def gamma_correction(img, gamma):
    """
    ガンマ補正
    参考：https://newtechnologylifestyle.net/opnecv%E3%81%A7%E3%82%AC%E3%83%B3%E3%83%9E%E8%A3%9C%E6%AD%A3%E3%82%92%E8%A1%8C%E3%81%A3%E3%81%A6%E3%81%BF%E3%82%8B/

    Parameters
    ----------
    img : numpy (OpenCV image)
        入力画像
    gamma : floag
        ガンマ補正で使用するパラメタ

    Returns
    -------
    img : numpy (OpenCV image)
        ガンマ補正後の画像
    """
    gamma_cvt = np.zeros((256, 1), dtype='uint8')
    for i in range(256):
        gamma_cvt[i][0] = 255 * (float(i) / 255) ** (1.0 / gamma)
    img = cv2.LUT(img, gamma_cvt)
    return img


def adjust_contrust(img, coefficient):
    """
    コントラスト調整
    参考：https://qiita.com/hmuronaka/items/008de58a8c21e8dd98e7

    Parameters
    ----------
    img : numpt (OpenCV image)
        入力画像
    coefficient : float
        シグモイド曲線の補正に使用する係数

    Returns
    -------
    result_img : numpy (OpenCV image)
        コントラスト調整後の画像
    """
    lut = [np.uint8(255.0 / (1 + math.exp(-coefficient * (i - 128.) / 255.)))
           for i in range(256)]
    result_img = np.array(
        [lut[value] for value in img.flat],
        dtype=np.uint8)
    result_img = result_img.reshape(img.shape)
    return result_img


def adjust_objects_image(
    img,
    gamma,
    contrast_coefficient,
    fast_nl_means_denoising_h):
    """
    テンプレートマッチング用の補正処理

    Parameters
    ----------
    img : numpy (OpenCV image)
        入力画像
    gamma : float
        ガンマ補正で使用するパラメタ
    contrast_coefficient : float
        コントラスト補正で，シグモイド曲線の補正に使用する係数
    fast_nl_means_denoising_h : float
        Fast Ni Means Denoising法にようるノイズ除去で用いる係数「h」

    Returns
    -------
    img : numpy (OpenCV image)
        補正処理後の画像
    """
    img = gamma_correction(img, gamma)
    img = adjust_contrust(img, contrast_coefficient)
    img = cv2.adaptiveThreshold(
        img,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11,
        2)
    img = cv2.fastNlMeansDenoising(img, h=fast_nl_means_denoising_h)
    return img

